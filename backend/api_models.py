"""Request bodies for the REST API, validated by pydantic."""

from typing import Dict, List, Optional

from pydantic import BaseModel, Field


class DriveBody(BaseModel):
    walk_x: Optional[float] = Field(None, ge=-1.0, le=1.0)
    walk_y: Optional[float] = Field(None, ge=-1.0, le=1.0)
    turn: Optional[float] = Field(None, ge=-1.0, le=1.0)
    speed: Optional[float] = Field(None, ge=0.0, le=1.0)
    step_height: Optional[float] = Field(None, ge=0.0, le=90.0)
    stride: Optional[float] = Field(None, ge=0.0, le=1.5)


class GaitBody(BaseModel):
    name: str


class StanceBody(BaseModel):
    name: str


class BodyPoseBody(BaseModel):
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None


class AxisBody(BaseModel):
    """A two-axis pad reading, interpreted through the selected axis mode."""
    mode: Optional[str] = None
    h: float = Field(0.0, ge=-1.0, le=1.0)
    v: float = Field(0.0, ge=-1.0, le=1.0)


class BehaviourBody(BaseModel):
    torque: Optional[bool] = None
    hold_pos: Optional[bool] = None
    auto_level: Optional[bool] = None
    auto_sit: Optional[bool] = None


class JointsBody(BaseModel):
    coxa: float
    femur: float
    tibia: float


class AllLegsJointBody(BaseModel):
    joint: str
    value: float = 0.0


class FootBody(BaseModel):
    x: float
    y: float
    z: float


class CalibrationFitBody(BaseModel):
    leg: Optional[str] = None
    joint: Optional[str] = None


class CalibrationBody(BaseModel):
    leg: str
    joint: str
    offset: Optional[float] = None
    direction: Optional[float] = None
    min: Optional[float] = Field(None, alias="min_deg")
    max: Optional[float] = Field(None, alias="max_deg")

    model_config = {"populate_by_name": True}


class PoseSaveBody(BaseModel):
    name: str


class TiltBody(BaseModel):
    """Tilt the simulated ground plane, to exercise auto-level with no robot."""
    roll: float = Field(0.0, ge=-35.0, le=35.0)
    pitch: float = Field(0.0, ge=-35.0, le=35.0)


class BeepBody(BaseModel):
    tone: str = "beep"
