
from pitop.core.import_opencv import import_opencv

cv2 = import_opencv()

def get_color_mask(frame, color: str):
    blurred = cv2.blur(frame, (11, 11))
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    masks = []
    color_ranges = {
        "red": [
            {"lower": (150, 100, 100), "upper": (179, 255, 255)},
            {"lower": (0, 100, 100), "upper": (5, 255, 255)},
        ],
        "orange": [{"lower": (10, 100, 20), "upper": (25, 255, 255)}],
        "green": [{"lower": (60, 100, 100), "upper": (90, 255, 255)}],
        # "blue": [{"lower": (100, 100, 100), "upper": (130, 255, 255)}],
        "blue": [{"lower": (110, 150, 150), "upper": (130, 255, 255)}],
    }

    for color_range in color_ranges[color]:
        hsv_lower = color_range["lower"]
        hsv_upper = color_range["upper"]
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        masks.append(mask)
    mask = sum(masks)

    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    return mask