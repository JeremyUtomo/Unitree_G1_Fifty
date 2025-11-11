from __future__ import annotations

import logging
import time
from typing import Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)


def _rpc_get_int(client: LocoClient, api_id: int) -> Optional[int]:
    try:
        code, data = client._Call(api_id, "{}")  # type: ignore[attr-defined]
        if code == 0 and data:
            import json

            return json.loads(data).get("data")
    except Exception:
        pass
    return None


def _fsm_id(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_ID)


def _fsm_mode(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_MODE)


def hanger_boot_sequence(
    iface: str = "lo",
    step: float = 0.02,
    max_height: float = 0.5,
    logger: Optional[logging.Logger] = None,
) -> LocoClient:
    
    print(iface)

    if logger is None:
        logging.basicConfig(level=logging.INFO, format="%(message)s")
        logger = logging.getLogger("hanger_boot")

    ChannelFactoryInitialize(0, iface)

    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    try:
        current_id = _fsm_id(sport_client)
        current_mode = _fsm_mode(sport_client)

        if current_id == 200 and current_mode is not None and current_mode != 2:
            logger.info(
                "Robot already in balanced stand (FSM 200, mode %s) – skipping boot sequence.",
                current_mode,
            )

            return sport_client
    except Exception:
        pass

    def show(tag: str) -> None:
        logger.info("%-12s → FSM %s   mode %s", tag, _fsm_id(sport_client), _fsm_mode(sport_client))

    sport_client.Damp(); show("damp")

    sport_client.SetFsmId(4); show("stand_up")

    while True:
        height = 0.0

        while height < max_height:
            height += step
            sport_client.SetStandHeight(height)
            show(f"height {height:.2f} m")

            if _fsm_mode(sport_client) == 0 and height > 0.2:
                break

        if _fsm_mode(sport_client) == 0:
            break

        logger.warning(
            "Feet still unloaded (mode %s) after reaching %.2f m.\n"
            "Adjust hanger height (raise/lower until the soles are just in\n"
            "contact with the ground) then press <Enter> to try again…",
            _fsm_mode(sport_client),
            height,
        )

        try:
            sport_client.SetStandHeight(0.0)
            show("reset")
        except Exception:
            pass

        input()

    sport_client.BalanceStand(0); show("balance")
    sport_client.SetStandHeight(height); show("height✔")

    sport_client.Start(); show("start")

    return sport_client


__all__ = ["hanger_boot_sequence"]