# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import importlib.util
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "limo_preflight", Path(__file__).resolve().parents[1] / "scripts" / "limo_preflight.py"
)
limo_preflight = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(limo_preflight)  # type: ignore[union-attr]

summarize_scope = limo_preflight.summarize_scope
ITM, YOLO, SAM, GDINO, VQA, ATTR = (
    limo_preflight.ITM,
    limo_preflight.YOLO,
    limo_preflight.SAM,
    limo_preflight.GDINO,
    limo_preflight.VQA,
    limo_preflight.ATTR,
)

_ALL_DEPS = dict(torch_ok=True, rclpy_ok=True, nav2_ok=True)


def test_full_g3_when_everything_up() -> None:
    # Real cloud verify = attr-verifier :12186 (NOT BLIP2-VQA :12185).
    ports = {ITM: True, YOLO: True, SAM: True, ATTR: True}
    scope = summarize_scope(ports, **_ALL_DEPS)
    assert scope["attribute_verify"] is True
    assert scope["level"].startswith("G3: full")
    assert scope["missing"] == []


def test_this_box_state_is_g2_lite() -> None:
    # ITM + YOLO up, SAM/VQA down, torch present -> exploration only.
    ports = {ITM: True, YOLO: True, SAM: False, VQA: False, ATTR: False}
    scope = summarize_scope(ports, **_ALL_DEPS)
    assert scope["value_map_explore"] is True
    assert scope["object_navigate"] is False
    assert scope["level"].startswith("G2-lite")
    assert any("MobileSAM" in m for m in scope["missing"])


def test_missing_torch_blocks_everything() -> None:
    ports = {ITM: True, YOLO: True, SAM: True, VQA: True}
    scope = summarize_scope(ports, torch_ok=False, rclpy_ok=True, nav2_ok=True)
    assert scope["value_map_explore"] is False
    assert scope["level"].startswith("blocked")
    assert any("torch" in m for m in scope["missing"])


def test_object_navigate_without_cloud_verify_uses_heuristic() -> None:
    # SAM up but attr-verifier down: full G3 flow still runs via local heuristic.
    ports = {ITM: True, YOLO: True, SAM: True, ATTR: False}
    scope = summarize_scope(ports, **_ALL_DEPS)
    assert scope["object_navigate"] is True
    assert scope["attribute_verify"] is False
    assert scope["level"].startswith("G3 (heuristic verify)")
    assert any(str(ATTR) in m for m in scope["missing"])


def test_attr_verifier_cloud_satisfies_verify() -> None:
    ports = {ITM: True, YOLO: True, SAM: True, ATTR: True}
    scope = summarize_scope(ports, **_ALL_DEPS)
    assert scope["attribute_verify"] is True


def test_vqa_is_not_required_for_verify() -> None:
    # BLIP2-VQA (:12185) down must not affect the verify milestone.
    ports = {ITM: True, YOLO: True, SAM: True, ATTR: True, VQA: False}
    scope = summarize_scope(ports, **_ALL_DEPS)
    assert scope["attribute_verify"] is True
    assert not any(str(VQA) in m for m in scope["missing"])


def test_non_coco_target_needs_grounding_dino() -> None:
    # YOLO up but GDINO down: a non-COCO target cannot be detected.
    ports = {ITM: True, YOLO: True, GDINO: False, SAM: True}
    scope = summarize_scope(ports, target_is_coco=False, **_ALL_DEPS)
    assert scope["value_map_explore"] is False
    assert any(str(GDINO) in m for m in scope["missing"])
