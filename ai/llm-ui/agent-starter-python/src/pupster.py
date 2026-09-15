"""Pupster: the LiveKit Agent (persona + tools) and its session builders.

The tools are deliberately few. Motion is expressed as ONE plan handed to the
executor via `execute_plan`; everything the robot says about motion comes back
from executor state (`status`) or executor events (PlanWatcher), never from the
model's belief about what it asked for.
"""

import json
import logging
import subprocess
from pathlib import Path

from livekit.agents import Agent, AgentSession, RunContext
from livekit.agents.llm import function_tool
from livekit.plugins import cartesia, google, openai
from openai.types import realtime
from openai.types.beta.realtime.session import TurnDetection

from animations import ANIMATION_NAMES, get_animation_duration  # noqa: F401 - re-exported

logger = logging.getLogger("agent")


def load_system_prompt() -> str:
    path = Path(__file__).parent / "system_prompt.md"
    with open(path, "r", encoding="utf-8") as f:
        prompt = f.read().strip()
    logger.info("loaded system prompt from %s (%d chars)", path, len(prompt))
    return prompt


########### VOICE OPTIONS ###############
# Sacha baren cohen (unintentional)
# tts=cartesia.TTS(voice="66db04d7-bca8-43dc-bf55-d432e4469b07", model="sonic-2")
# Dug
# tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-2"),
# Nathan
# tts=cartesia.TTS(voice="70e274d6-3e98-49bf-b482-f7374b045dc8", model="sonic-2"),
# Teresa
# tts=cartesia.TTS(voice="47836b34-00be-4ada-bec2-9b69c73304b5", model="sonic-2"),
# Spanish
# tts=cartesia.TTS(voice="79743797-422f-8dc7-86f9efca85f1", model="sonic-2"),
#########################################


def openairealtime_session():
    return AgentSession(
        max_tool_steps=2,
        llm=openai.realtime.RealtimeModel(
            modalities=["audio"],
            model="gpt-realtime",
            voice="echo",
            turn_detection=realtime.realtime_audio_input_turn_detection.SemanticVad(
                type="semantic_vad",
                eagerness="high",
                create_response=True,
                interrupt_response=False,
            ),
            input_audio_noise_reduction="far_field",
            input_audio_transcription=realtime.AudioTranscription(
                model="gpt-4o-transcribe",
                language="en",
            ),
        ),
    )


def openairealtime_cartesia_session():
    return AgentSession(
        max_tool_steps=2,
        llm=openai.realtime.RealtimeModel(
            modalities=["text"],
            model="gpt-realtime",
            turn_detection=TurnDetection(
                type="server_vad",
                threshold=0.7,
                prefix_padding_ms=200,
                silence_duration_ms=100,
                create_response=True,
                interrupt_response=True,
            ),
        ),
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9", model="sonic-3"),
    )


def gemini_cartesia_session():
    return AgentSession(
        max_tool_steps=2,
        llm=google.beta.realtime.RealtimeModel(
            model="gemini-live-2.5-flash-preview",
            voice="Puck",
            temperature=0.8,
            instructions="",
            modalities=["text"],
        ),
        tts=cartesia.TTS(voice="e7651bee-f073-4b79-9156-eff1f8ae4fd9"),
    )


def get_pupster_session(agent_design: str):
    if agent_design == "cascade":
        # return cascaded_session()
        raise NotImplementedError("Cascade session is currently disabled due to VAD model issues on images.")
    elif agent_design == "google-cartesia":
        return gemini_cartesia_session()
    elif agent_design == "openai-cartesia":
        return openairealtime_cartesia_session()
    elif agent_design == "openai-realtime":
        return openairealtime_session()
    else:
        logger.error(f"Unknown agent design {agent_design}")
        raise ValueError(f"Unknown agent design {agent_design}")


EXECUTE_PLAN_DOC = """Hand the robot ONE plan to execute. This is the only way to move.

Call it exactly once per movement command, with nothing else in that response, and
speak only after it returns. It returns "Accepted: ..." (the robot is now doing it;
the executor will report when done), "Done: ..." (it finished immediately), or
"Rejected: <reason>" (fix the plan once, resend, then explain if it fails again).

plan_json is a JSON string:
{"v":1,"goal":"<the user's words>","mode":"replace","on_fail":"report",
 "steps":[{"id":"s1","skill":"<skill>","args":{...},"timeout_s":<optional 1-120>}, ...]}

Skills and args (all distances in metres, angles in degrees, at most 12 steps):
- turn {degrees: -360..360, speed_dps?: 20..120}   degrees > 0 = LEFT, < 0 = RIGHT. 180 = turn around.
- move {meters: -5..5, lateral_m?: -5..5, speed_mps?: 0.35..0.75}   meters > 0 = forward, lateral_m > 0 = LEFT.
- return_to_start {restore_heading?: bool}   walk back to where this plan began. Use for "and back", "come back", "return".
- go_to_object {label: "bed", stop_distance_m?: 0.4..3.0, search?: bool}   looks, turns toward it, walks up to it, searches around if not visible.
- find_object {label, search_step_deg?: 30..120}   rotate in steps until the object is seen (no walking).
- go_to_pose {x, y, yaw_deg?, frame?: "start"|"odom"}
- look {prompt}   one camera look inside a plan (use ask_scene for questions instead).
- wait {seconds: 0..30}
- say {text: <=200 chars}   the robot speaks this when the step is reached.
- animate {name}   one of the animation names.
- stop {}
- find_person {who: "<name>"|"nearest"}   (people features may be unavailable; expect a rejection)
- follow_person {who: "<name>"|"nearest", target_range_m?}   open-ended until stop().

Spatial phrases: "behind you" = turn 180 FIRST, then the rest. "to your left" = turn 90; "to your right" = turn -90.
"and back" / "then come back" = append return_to_start. "turn around" = turn 180.

Example, "walk to the bed and back":
{"v":1,"goal":"walk to the bed and back","mode":"replace","on_fail":"report",
 "steps":[{"id":"s1","skill":"go_to_object","args":{"label":"bed","stop_distance_m":0.8},"timeout_s":90},
          {"id":"s2","skill":"return_to_start","args":{"restore_heading":true}},
          {"id":"s3","skill":"say","args":{"text":"Back where I started."}}]}

Args:
    plan_json (str): the plan as a JSON string, exactly in the format above.
"""


# Tool parameter schema = the plan itself (a permissive mirror of pupper_interfaces/schema/plan_v1.json; the
# executor enforces the strict version and returns a precise "Rejected: ..." the model can fix).
EXECUTE_PLAN_SCHEMA = {
    "name": "execute_plan",
    "description": EXECUTE_PLAN_DOC,
    "parameters": {
        "type": "object",
        "properties": {
            "v": {"type": "integer", "enum": [1]},
            "goal": {"type": "string", "description": "the user's words"},
            "mode": {"type": "string", "enum": ["replace", "queue"]},
            "on_fail": {"type": "string", "enum": ["report", "return_to_start_then_report"]},
            "steps": {
                "type": "array",
                "minItems": 1,
                "maxItems": 12,
                "items": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "skill": {
                            "type": "string",
                            "enum": ["turn", "move", "go_to_pose", "return_to_start", "go_to_object", "find_object",
                                     "look", "wait", "say", "animate", "relax", "stop", "find_person", "follow_person"],
                        },
                        "args": {"type": "object", "description": "skill arguments, see the skill table"},
                        "timeout_s": {"type": "number"},
                    },
                    "required": ["id", "skill", "args"],
                },
            },
        },
        "required": ["v", "goal", "steps"],
    },
}

class PupsterAgent(Agent):
    def __init__(self, bridge) -> None:
        super().__init__(instructions=load_system_prompt())
        self.bridge = bridge

    async def on_enter(self) -> None:
        logger.info("PupsterAgent on_enter")
        try:
            Path("/tmp/pupster_agent_started").touch()
        except OSError as e:
            logger.warning("could not touch start marker: %s", e)
        chat_ctx = self.chat_ctx.copy()
        chat_ctx.add_message(role="system", content="Say hi to the user and introduce yourself as Pupster, in one sentence.")
        await self.update_chat_ctx(chat_ctx)
        self.session.generate_reply()

    @function_tool(raw_schema=EXECUTE_PLAN_SCHEMA)
    async def execute_plan(self, raw_arguments: dict[str, object], context: RunContext) -> str:
        # Raw-schema tool: the model sends the plan object itself as the arguments (it refused to wrap it
        # in a single string field), and the executor validates it against the full plan_v1 schema.
        plan_json = json.dumps(raw_arguments, separators=(",", ":"))
        logger.info("FUNCTION CALL: execute_plan(%s)", plan_json)
        return await self.bridge.execute_plan(plan_json)

    @function_tool
    async def stop(self, context: RunContext) -> str:
        """Stop all motion immediately and cancel the running plan. Call this for "stop", "halt", "wait", "freeze", "no", or anything that means stop. Returns what the robot is doing now."""
        logger.info("FUNCTION CALL: stop()")
        return await self.bridge.stop()

    @function_tool
    async def status(self, context: RunContext) -> str:
        """What the robot is actually doing right now, from the executor: idle, or which step of which plan, and where it is relative to its start. Use this to answer "what are you doing", "are you following me", "where are you". Never guess; call this."""
        logger.info("FUNCTION CALL: status()")
        return await self.bridge.status()

    @function_tool
    async def ask_scene(self, context: RunContext, prompt: str) -> str:
        """Look through the camera and describe the scene, answering the prompt. Returns text plus objects with headings (left/right) and rough distances. Use for "what do you see", "is there a X", "describe the room". Does not move the robot.

        Args:
            prompt (str): what to look for or describe, e.g. "Describe the room and list the furniture".
        """
        logger.info("FUNCTION CALL: ask_scene(%s)", prompt)
        return await self.bridge.ask_scene(prompt)

    @function_tool(
        description="Play a pre-recorded animation (a trick). Blocks until it finishes. Available animations:\n"
        + "\n".join(f'- "{name}": {data["description"]}' for name, data in ANIMATION_NAMES.items())
        + "\n\nArgs:\n    animation_name (str): one of the names above."
    )
    async def animate(self, context: RunContext, animation_name: str) -> str:
        logger.info("FUNCTION CALL: animate(%s)", animation_name)
        return await self.bridge.animate(animation_name)

    @function_tool
    async def set_speaker_volume(self, context: RunContext, volume: int) -> str:
        """Set the speaker volume.

        Args:
            volume (int): 0 to 150. off=0, very quiet=50, quiet=75, normal=100, loud=125, very loud=150. Only go below 50 if asked to be silent.
        """
        logger.info("FUNCTION CALL: set_speaker_volume(%s)", volume)
        try:
            level = max(0.0, min(volume / 150 * 1.5, 1.5))
            subprocess.run(["wpctl", "set-volume", "@DEFAULT_SINK@", str(level)], check=True)
            return f"Speaker volume set to {volume}%."
        except Exception as e:  # noqa: BLE001
            logger.error("set_speaker_volume failed: %s", e)
            return f"Failed to set speaker volume: {e}"

    @function_tool
    async def power(self, context: RunContext, mode: str) -> str:
        """Turn the motors on (stand up, ready to walk) or off (lie down, relax). Plans activate the motors by themselves, so only call this when the user explicitly asks to power on/off, stand up, lie down, relax, or shut down.

        Args:
            mode (str): "activate" or "deactivate".
        """
        logger.info("FUNCTION CALL: power(%s)", mode)
        return await self.bridge.power(mode)

    @function_tool
    async def follow_me(self, context: RunContext) -> str:
        """Start following the person in front of the camera. Only claim to be following if the result says so; if nobody is visible the robot will not move."""
        logger.info("FUNCTION CALL: follow_me()")
        return await self.bridge.follow_me()

    @function_tool
    async def stop_following(self, context: RunContext) -> str:
        """Stop following a person."""
        logger.info("FUNCTION CALL: stop_following()")
        return await self.bridge.stop_following()
