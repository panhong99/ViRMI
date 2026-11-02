"""
YOLOE 단독 multi-object detection 테스트 (RealSense D455)
- prompt 모드: 텍스트로 지정한 클래스만 detect (plate, baseball, rubik's cube 등)
- prompt-free 모드: 클래스 지정 없이 내장 vocab으로 detect (-pf 가중치 사용)

설치:
    pip install -U ultralytics pyrealsense2 opencv-python numpy

모델 가중치는 최초 실행 시 자동 다운로드됨.
직접 받고 싶으면: https://huggingface.co/jameslahm/yoloe

사용법:
    # prompt 모드 (기본): 지정한 클래스만 탐지
    python experiments/yoloe/check_yoloe.py --source realsense --mode prompt --classes plate baseball "rubik's cube"

    # prompt-free 모드: 클래스 지정 없이 내장 vocab 전체 탐지
    python experiments/yoloe/check_yoloe.py --source realsense --mode promptfree

    # 정지 이미지로 테스트
    python experiments/yoloe/check_yoloe.py --source test.jpg --mode prompt --classes plate

    # 일반 웹캠(카메라 index)으로 테스트
    python experiments/yoloe/check_yoloe.py --source 0
"""

import argparse
import time

import cv2
import numpy as np
from ultralytics import YOLOE

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


def resolve_model_name(model_name: str, mode: str) -> str:
    # prompt-free 모드는 전용 -pf 가중치 필요 (예: yoloe-v8l-seg.pt -> yoloe-v8l-seg-pf.pt)
    if mode != "promptfree" or model_name.endswith("-pf.pt"):
        return model_name
    return model_name.replace(".pt", "-pf.pt")


def build_model(model_name: str, mode: str, classes: list[str]):
    model = YOLOE(model_name)
    if mode == "prompt":
        text_pe = model.get_text_pe(classes)
        model.set_classes(classes, text_pe)
    return model


class RealSenseCamera:
    def __init__(self, width: int, height: int, fps: int, serial: str = ''):
        if rs is None:
            raise RuntimeError("pyrealsense2가 설치되어 있지 않음: pip install pyrealsense2")
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:                                   # two D455s: fixed 338122300585, wrist 338122303684
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(config)

    def read(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        return True, np.asanyarray(color_frame.get_data())

    def release(self):
        self.pipeline.stop()


def run_on_image(model, image_path: str, conf: float):
    results = model.predict(source=image_path, conf=conf, verbose=False)
    r = results[0]
    print(f"\n[검출 결과] {len(r.boxes)}개 object 검출됨")
    for box in r.boxes:
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]
        conf_val = float(box.conf[0])
        xyxy = box.xyxy[0].tolist()
        print(f"  - {cls_name}: conf={conf_val:.2f}, bbox={[round(v, 1) for v in xyxy]}")

    annotated = r.plot()
    out_path = "yoloe_result.jpg"
    cv2.imwrite(out_path, annotated)
    print(f"\n결과 이미지 저장: {out_path}")


def run_on_stream(model, cap, conf: float, window_name: str, window_size: tuple[int, int]):
    prev_t = time.time()
    fps_smooth = 0.0

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, *window_size)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model.predict(source=frame, conf=conf, verbose=False)
        r = results[0]
        annotated = r.plot()

        now = time.time()
        inst_fps = 1.0 / max(now - prev_t, 1e-6)
        fps_smooth = inst_fps if fps_smooth == 0 else 0.9 * fps_smooth + 0.1 * inst_fps
        prev_t = now

        cv2.putText(
            annotated, f"FPS: {fps_smooth:.1f}  objects: {len(r.boxes)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
        )
        cv2.imshow(window_name, annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", required=True,
        help="'realsense'(D455) / 카메라 index(예: 0) / 이미지 경로",
    )
    parser.add_argument(
        "--mode", choices=["prompt", "promptfree"], default="prompt",
        help="prompt: --classes로 지정한 클래스만 탐지 / promptfree: 내장 vocab 전체 탐지 (-pf 가중치 사용)",
    )
    parser.add_argument(
        "--model", default="yoloe-v8l-seg.pt",
        help="yoloe-v8s-seg.pt(빠름) / yoloe-v8m-seg.pt / yoloe-v8l-seg.pt(정확). "
             "promptfree 모드면 자동으로 -pf 붙은 가중치로 치환됨",
    )
    parser.add_argument(
        "--classes", nargs="+",
        default=["plate", "baseball", "rubik's cube"],
        help="prompt 모드에서 detect할 클래스 이름 리스트",
    )
    parser.add_argument("--serial", default="", help="realsense serial (empty = first device)")
    parser.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--width", type=int, default=640, help="realsense color width")
    parser.add_argument("--height", type=int, default=480, help="realsense color height")
    parser.add_argument("--fps", type=int, default=30, help="realsense color fps")
    parser.add_argument("--window-width", type=int, default=1280, help="표시 창 가로 크기")
    parser.add_argument("--window-height", type=int, default=960, help="표시 창 세로 크기")
    args = parser.parse_args()

    model_name = resolve_model_name(args.model, args.mode)
    print(f"모드: {args.mode}")
    print(f"모델: {model_name}")
    if args.mode == "prompt":
        print(f"클래스: {args.classes}")
    model = build_model(model_name, args.mode, args.classes)

    window_size = (args.window_width, args.window_height)
    if args.source == "realsense":
        cam = RealSenseCamera(args.width, args.height, args.fps, args.serial)
        run_on_stream(model, cam, args.conf, "YOLOE D455 test (q to quit)", window_size)
    elif args.source.isdigit():
        cap = cv2.VideoCapture(int(args.source))
        if not cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없음: {args.source}")
        run_on_stream(model, cap, args.conf, "YOLOE webcam test (q to quit)", window_size)
    else:
        run_on_image(model, args.source, args.conf)


if __name__ == "__main__":
    main()
