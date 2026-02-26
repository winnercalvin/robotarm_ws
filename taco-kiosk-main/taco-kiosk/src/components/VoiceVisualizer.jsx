import React from "react";

export default function VoiceVisualizer({ status }) {
  // 상태에 따른 메시지 및 색상
  let message = "말씀하시면 주문을 도와드려요!";
  let color = "#aaa";
  let pulse = false;

  if (status === "IDLE") {
    message = "'헤이 타코'라고 불러보세요!";
    color = "#555";
  } else {
    message = "듣고 있어요...";
    color = "#d9381e"; // 타코 레드
    pulse = true;
  }

  return (
    <div
      style={{
        position: "fixed",
        bottom: "300px", // 하단 바 위쪽에 위치
        right: "20px",
        zIndex: 1000,
        display: "flex",
        flexDirection: "column",
        alignItems: "end",
      }}
    >
      {/* 말풍선 */}
      <div
        style={{
          backgroundColor: "white",
          padding: "10px 15px",
          borderRadius: "15px 15px 0 15px",
          boxShadow: "0 2px 10px rgba(0,0,0,0.1)",
          marginBottom: "10px",
          fontSize: "0.9rem",
          fontWeight: "bold",
          color: "#333",
        }}
      >
        {message}
      </div>

      {/* 마이크 아이콘 */}
      <div
        style={{
          width: "60px",
          height: "60px",
          borderRadius: "50%",
          backgroundColor: color,
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          boxShadow: "0 4px 15px rgba(0,0,0,0.2)",
          color: "white",
          fontSize: "1.5rem",
          animation: pulse ? "pulse 1.5s infinite" : "none",
        }}
      >
        🎙️
      </div>

      {/* CSS 애니메이션 (index.css에 넣어도 됨) */}
      <style>{`
        @keyframes pulse {
          0% { box-shadow: 0 0 0 0 rgba(217, 56, 30, 0.7); }
          70% { box-shadow: 0 0 0 15px rgba(217, 56, 30, 0); }
          100% { box-shadow: 0 0 0 0 rgba(217, 56, 30, 0); }
        }
      `}</style>
    </div>
  );
}
