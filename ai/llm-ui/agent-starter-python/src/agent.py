"""Pupster voice agent entrypoint.

    python3 src/agent.py console                      # on the robot (ROS bridge)
    python3 src/agent.py --tool-server nop console    # laptop, no robot, fake executor
"""

import argparse
import asyncio
import logging
import os
import sys
import time
from datetime import datetime, timezone

logging.getLogger("livekit.agents").setLevel(logging.WARNING)
logging.getLogger("livekit").setLevel(logging.WARNING)

from dotenv import load_dotenv  # noqa: E402
from livekit.agents import (  # noqa: E402
    NOT_GIVEN,
    AgentFalseInterruptionEvent,
    AgentStateChangedEvent,
    ConversationItemAddedEvent,
    JobContext,
    JobProcess,
    MetricsCollectedEvent,
    RoomInputOptions,
    UserInputTranscribedEvent,
    UserStateChangedEvent,
    WorkerOptions,
    cli,
    llm,
    metrics,
)

from plan_watcher import PlanWatcher  # noqa: E402
from pupster import PupsterAgent, get_pupster_session  # noqa: E402

load_dotenv(".env.local")

logger = logging.getLogger("agent")

AGENT_DESIGN = "openai-realtime"


def lat(evt: str, key: str = "") -> None:
    logger.info("LAT evt=%s t=%d wall=%s key=%s", evt, time.monotonic_ns(), datetime.now(timezone.utc).isoformat(), key)


def make_bridge():
    kind = os.environ.get("PUPSTER_TOOL_SERVER", "ros")
    if kind == "nop":
        from nop_bridge import NopBridge

        logger.info("tool server: nop (no robot)")
        return NopBridge()
    from executor_bridge import ExecutorBridge

    logger.info("tool server: ros")
    return ExecutorBridge()


def prewarm(proc: JobProcess) -> None:
    if AGENT_DESIGN == "cascade":
        from livekit.plugins import silero

        proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext) -> None:
    ctx.log_context_fields = {"room": ctx.room.name}

    session = get_pupster_session(AGENT_DESIGN)
    bridge = make_bridge()
    watcher = PlanWatcher(session, bridge)
    watcher.start()

    @session.on("conversation_item_added")
    def on_conversation_item_added(event: ConversationItemAddedEvent) -> None:
        for content in event.item.content:
            if isinstance(content, str):
                logger.info("[%s] %s", event.item.role, content)
            elif isinstance(content, llm.AudioContent):
                logger.info("[%s audio] %s", event.item.role, content.transcript)

    @session.on("agent_false_interruption")
    def on_false_interruption(ev: AgentFalseInterruptionEvent) -> None:
        logger.info("false positive interruption, resuming")
        session.generate_reply(instructions=ev.extra_instructions or NOT_GIVEN)

    @session.on("agent_state_changed")
    def on_agent_state(ev: AgentStateChangedEvent) -> None:
        speaking = ev.new_state == "speaking"
        if speaking:
            lat("agent_speaking_start")
        try:
            bridge.set_speaking(speaking)
        except Exception:  # noqa: BLE001
            logger.exception("set_speaking")

    @session.on("user_state_changed")
    def on_user_state(ev: UserStateChangedEvent) -> None:
        if ev.new_state == "speaking":
            lat("user_speech_started")
            asyncio.ensure_future(bridge.prepare())

    @session.on("user_input_transcribed")
    def on_user_input_transcribed(ev: UserInputTranscribedEvent) -> None:
        if ev.is_final:
            lat("user_speech_stopped", ev.transcript[:40])
        logger.info("user transcript final=%s: %s", ev.is_final, ev.transcript)

    usage_collector = metrics.UsageCollector()

    @session.on("metrics_collected")
    def on_metrics_collected(ev: MetricsCollectedEvent) -> None:
        metrics.log_metrics(ev.metrics)
        usage_collector.collect(ev.metrics)

    async def log_usage() -> None:
        logger.info("Usage: %s", usage_collector.get_summary())

    async def shutdown_bridge() -> None:
        bridge.shutdown()

    ctx.add_shutdown_callback(log_usage)
    ctx.add_shutdown_callback(shutdown_bridge)

    await session.start(
        agent=PupsterAgent(bridge=bridge),
        room=ctx.room,
        room_input_options=RoomInputOptions(),
    )
    await ctx.connect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--tool-server", choices=["ros", "nop"], dest="tool_server", default=None)
    args, passthrough = parser.parse_known_args(sys.argv[1:])
    if args.tool_server:
        os.environ["PUPSTER_TOOL_SERVER"] = args.tool_server
    sys.argv = [sys.argv[0]] + passthrough
    cli.run_app(WorkerOptions(entrypoint_fnc=entrypoint, prewarm_fnc=prewarm))
