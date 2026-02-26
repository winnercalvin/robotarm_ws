import React, { useRef, useState, useEffect } from "react";

export default function SauceCanvas({ onSave, onClose }) {
  const canvasRef = useRef(null);
  const [isDrawing, setIsDrawing] = useState(false);

  // 로봇에게 보낼 좌표 데이터 (순서대로 저장됨 -> 방향성 포함)
  const [pathData, setPathData] = useState([]);

  // 캔버스 설정
  useEffect(() => {
    const canvas = canvasRef.current;
    canvas.width = 300; // 그리기 영역 크기
    canvas.height = 300;

    const ctx = canvas.getContext("2d");
    ctx.lineCap = "round";
    ctx.strokeStyle = "#d9381e"; // 소스 색깔 (빨간색)
    ctx.lineWidth = 5;
    ctx.fillStyle = "#fff";
    ctx.fillRect(0, 0, canvas.width, canvas.height); // 흰 배경
  }, []);

  // --- 그리기 로직 (마우스/터치 통합) ---
  const getPos = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();

    // 터치 이벤트인지 마우스 이벤트인지 확인
    const clientX = e.touches ? e.touches[0].clientX : e.clientX;
    const clientY = e.touches ? e.touches[0].clientY : e.clientY;

    return {
      x: clientX - rect.left,
      y: clientY - rect.top,
    };
  };

  const startDrawing = (e) => {
    e.preventDefault(); // 스크롤 방지
    setIsDrawing(true);
    const { x, y } = getPos(e);

    const ctx = canvasRef.current.getContext("2d");
    ctx.beginPath();
    ctx.moveTo(x, y);

    // 시작점 저장 (정규화: 0.0 ~ 1.0)
    savePoint(x, y);
  };

  const draw = (e) => {
    if (!isDrawing) return;
    e.preventDefault();
    const { x, y } = getPos(e);

    const ctx = canvasRef.current.getContext("2d");
    ctx.lineTo(x, y);
    ctx.stroke();

    // 이동 경로 저장 (데이터가 너무 많아지지 않게 드문드문 저장하려면 throttle 적용 가능)
    savePoint(x, y);
  };

  const endDrawing = () => {
    setIsDrawing(false);
    const ctx = canvasRef.current.getContext("2d");
    ctx.closePath();
  };

  // --- 좌표 저장 (로봇용 데이터 변환) ---
  const savePoint = (x, y) => {
    const canvas = canvasRef.current;
    // 픽셀 좌표를 0.0 ~ 1.0 비율로 변환 (Normalization)
    // 예: 폭 300px 중 150px 위치라면 -> 0.5
    const normalizedPoint = {
      x: (x / canvas.width).toFixed(3), // 소수점 3자리까지
      y: (y / canvas.height).toFixed(3),
      timestamp: Date.now(), // 속도 계산용 시간
    };
    setPathData((prev) => [...prev, normalizedPoint]);
  };

  const handleClear = () => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#fff";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    setPathData([]); // 데이터 초기화
  };

  return (
    <div style={modalStyles.overlay}>
      <div style={modalStyles.content}>
        <h3>🎨 나만의 소스 패턴 그리기</h3>
        <p style={{ fontSize: "0.9rem", color: "#666" }}>
          이곳에 그리면 로봇이 똑같이 뿌려줍니다!
        </p>

        <canvas
          ref={canvasRef}
          style={{
            border: "2px dashed #ccc",
            borderRadius: "10px",
            touchAction: "none",
          }}
          onMouseDown={startDrawing}
          onMouseMove={draw}
          onMouseUp={endDrawing}
          onMouseLeave={endDrawing}
          onTouchStart={startDrawing}
          onTouchMove={draw}
          onTouchEnd={endDrawing}
        />

        <div
          style={{
            marginTop: "20px",
            display: "flex",
            gap: "10px",
            justifyContent: "center",
          }}
        >
          <button onClick={handleClear} style={btnStyles.secondary}>
            지우기
          </button>
          <button onClick={onClose} style={btnStyles.secondary}>
            취소
          </button>
          <button onClick={() => onSave(pathData)} style={btnStyles.primary}>
            저장하기 ({pathData.length}개 점)
          </button>
        </div>
      </div>
    </div>
  );
}

// 간단한 인라인 스타일
const modalStyles = {
  overlay: {
    position: "fixed",
    top: 0,
    left: 0,
    width: "100%",
    height: "100%",
    backgroundColor: "rgba(0,0,0,0.7)",
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1000,
  },
  content: {
    backgroundColor: "white",
    padding: "20px",
    borderRadius: "15px",
    textAlign: "center",
    width: "350px",
  },
};

const btnStyles = {
  primary: {
    padding: "10px 20px",
    background: "#d9381e",
    color: "white",
    border: "none",
    borderRadius: "8px",
    fontWeight: "bold",
  },
  secondary: {
    padding: "10px 20px",
    background: "#eee",
    color: "#333",
    border: "none",
    borderRadius: "8px",
  },
};
