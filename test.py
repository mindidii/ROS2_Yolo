from flask import Flask, Response
import cv2

app = Flask(__name__)

# 5001 포트 영상 입력
cap = cv2.VideoCapture("udp://0.0.0.0:5001", cv2.CAP_FFMPEG)

def generate():
    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() +
            b'\r\n'
        )

@app.route('/')
def index():
    return """
    <html>
    <body>
        <h2>Video Stream</h2>
        <img src="/video_feed">
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, threaded=True)