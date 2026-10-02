"""Tests for metric panel mapping and multi-frame fusion."""

import math

from da_daka_control.panel_mapping import (
    camera_surface_distance,
    CameraGroundModel,
    MetricPanelObservation,
    nearest_timestamped_sample,
    PanelMapBuilder,
    PanelObservation,
    project_panel_observation,
    project_panel_observation_attitude,
    quaternion_tilt_rad,
)
import pytest


def test_nearest_timestamped_sample_uses_camera_capture_time():
    samples = [
        (1_000_000_000, 'before'),
        (1_080_000_000, 'capture-pose'),
        (1_240_000_000, 'arrival-pose'),
    ]
    assert nearest_timestamped_sample(
        samples,
        1_100_000_000,
        0.15,
    ) == 'capture-pose'


def test_nearest_timestamped_sample_rejects_unsynchronised_input():
    assert nearest_timestamped_sample(
        [(1_000_000_000, 'stale')],
        1_300_000_000,
        0.15,
    ) is None


def test_camera_surface_distance_applies_measured_vertical_offset():
    assert camera_surface_distance(3.0, -0.16) == pytest.approx(2.84)


def test_camera_surface_distance_rejects_nonpositive_result():
    with pytest.raises(ValueError, match='must be positive'):
        camera_surface_distance(0.10, -0.16)


def test_image_center_projects_below_vehicle():
    camera = CameraGroundModel(1.3, 0.8)
    result = project_panel_observation(
        PanelObservation(0.5, 0.5, 0.4, 0.3, 0.9),
        camera,
        vehicle_east_m=4.0,
        vehicle_north_m=-2.0,
        vehicle_yaw_rad=1.1,
        distance_m=3.0,
    )
    assert result.east_m == pytest.approx(4.0)
    assert result.north_m == pytest.approx(-2.0)
    assert result.width_m == pytest.approx(1.56)
    assert result.height_m == pytest.approx(0.72)


def test_image_top_projects_forward_and_rotates_with_yaw():
    camera = CameraGroundModel(1.0, 1.0)
    result = project_panel_observation(
        PanelObservation(0.5, 0.0, 0.2, 0.2, 1.0),
        camera,
        vehicle_east_m=0.0,
        vehicle_north_m=0.0,
        vehicle_yaw_rad=math.pi / 2.0,
        distance_m=2.0,
    )
    assert result.east_m == pytest.approx(0.0, abs=1e-8)
    assert result.north_m == pytest.approx(1.0)


def test_map_builder_merges_repeated_panel_and_rejects_one_frame_noise():
    builder = PanelMapBuilder(merge_radius_m=0.25, minimum_observations=2)
    first_id = builder.observe(
        MetricPanelObservation(1.0, 2.0, 1.2, 0.8, 0.8)
    )
    second_id = builder.observe(
        MetricPanelObservation(1.1, 2.0, 1.3, 0.9, 1.0)
    )
    builder.observe(MetricPanelObservation(5.0, 5.0, 1.2, 0.8, 0.9))

    targets = builder.targets()

    assert first_id == second_id
    assert len(targets) == 1
    assert targets[0].panel_id == first_id
    assert targets[0].observation_count == 2
    assert 1.0 < targets[0].east_m < 1.1


def test_last_night_postfilter_map_contains_two_stable_panels():
    """Regression for the two retained clusters from flight ULog 00119."""
    builder = PanelMapBuilder(merge_radius_m=0.45, minimum_observations=3)
    retained = (
        (-2.2725, -2.1839, 0.8955, 0.4433, 0.8925, 11),
        (-0.4662, -2.6829, 0.9699, 0.4998, 0.7768, 20),
    )
    offsets = ((-0.04, 0.02), (0.0, 0.0), (0.04, -0.02))
    for east, north, width, height, confidence, count in retained:
        for index in range(count):
            de, dn = offsets[index % len(offsets)]
            builder.observe(
                MetricPanelObservation(
                    east + de,
                    north + dn,
                    width,
                    height,
                    confidence,
                )
            )

    targets = builder.targets()

    assert len(targets) == 2
    assert [target.observation_count for target in targets] == [11, 20]
    assert math.dist(
        (targets[0].east_m, targets[0].north_m),
        (targets[1].east_m, targets[1].north_m),
    ) > 1.5


def test_full_attitude_projection_matches_level_yaw_projection():
    camera = CameraGroundModel(1.3, 0.8)
    observation = PanelObservation(0.75, 0.25, 0.2, 0.3, 0.9)
    yaw = math.pi / 3.0
    expected = project_panel_observation(
        observation,
        camera,
        vehicle_east_m=4.0,
        vehicle_north_m=-2.0,
        vehicle_yaw_rad=yaw,
        distance_m=3.0,
    )
    actual = project_panel_observation_attitude(
        observation,
        camera,
        vehicle_east_m=4.0,
        vehicle_north_m=-2.0,
        vehicle_up_m=3.0,
        vehicle_quaternion_xyzw=(
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        ),
        measured_center_distance_m=3.0,
    )
    assert actual.east_m == pytest.approx(expected.east_m)
    assert actual.north_m == pytest.approx(expected.north_m)
    assert actual.width_m == pytest.approx(expected.width_m)
    assert actual.height_m == pytest.approx(expected.height_m)


def test_full_attitude_projection_corrects_roll_for_image_centre():
    roll = math.radians(10.0)
    result = project_panel_observation_attitude(
        PanelObservation(0.5, 0.5, 0.2, 0.2, 1.0),
        CameraGroundModel(1.0, 1.0),
        vehicle_east_m=0.0,
        vehicle_north_m=0.0,
        vehicle_up_m=3.0,
        vehicle_quaternion_xyzw=(
            math.sin(roll / 2.0),
            0.0,
            0.0,
            math.cos(roll / 2.0),
        ),
        measured_center_distance_m=3.0,
    )
    assert result.east_m == pytest.approx(0.0, abs=1e-8)
    assert result.north_m == pytest.approx(3.0 * math.sin(roll))


def test_full_attitude_projection_rejects_camera_facing_horizon():
    pitch = math.pi / 2.0
    with pytest.raises(ValueError, match='does not point toward the ground'):
        project_panel_observation_attitude(
            PanelObservation(0.5, 0.5, 0.2, 0.2, 1.0),
            CameraGroundModel(1.0, 1.0),
            vehicle_east_m=0.0,
            vehicle_north_m=0.0,
            vehicle_up_m=3.0,
            vehicle_quaternion_xyzw=(
                0.0,
                math.sin(pitch / 2.0),
                0.0,
                math.cos(pitch / 2.0),
            ),
            measured_center_distance_m=3.0,
        )


def test_quaternion_tilt_is_yaw_independent_and_detects_roll():
    yaw_90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    roll_10 = (
        math.sin(math.radians(5.0)),
        0.0,
        0.0,
        math.cos(math.radians(5.0)),
    )
    assert quaternion_tilt_rad(yaw_90) == pytest.approx(0.0)
    assert quaternion_tilt_rad(roll_10) == pytest.approx(math.radians(10.0))
