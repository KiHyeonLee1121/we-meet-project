from laptop_ai.panel_detector import (
    PanelRectangle,
    select_panel_nearest_target,
)


def test_selects_centered_panel_instead_of_largest_panel():
    largest = PanelRectangle(1, 0, 0, 500, 300, 0.95)
    centered = PanelRectangle(2, 540, 300, 200, 120, 0.80)
    selected = select_panel_nearest_target(
        [largest, centered],
        image_width=1280,
        image_height=720,
        target_x_norm=0.5,
        target_y_norm=0.5,
        maximum_distance_norm=0.45,
    )
    assert selected == centered


def test_rejects_panels_outside_reacquisition_gate():
    selected = select_panel_nearest_target(
        [PanelRectangle(1, 0, 0, 100, 100, 0.9)],
        image_width=1280,
        image_height=720,
        target_x_norm=0.5,
        target_y_norm=0.5,
        maximum_distance_norm=0.2,
    )
    assert selected is None
