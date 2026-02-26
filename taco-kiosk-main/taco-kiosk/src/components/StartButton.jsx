import React from "react";

export default function StartButton({ onClick }) {
  return (
    <button
      onClick={onClick}
      style={styles.button}
      onMouseEnter={(e) => (e.target.style.transform = "scale(1.1)")} // 마우스 올리면 커짐
      onMouseLeave={(e) => (e.target.style.transform = "scale(1)")}
    >
      주문 시작하기
    </button>
  );
}

const styles = {
  button: {
    position: "absolute", // 이미지 위에 둥둥 띄우기 위함
    bottom: "15%", // 바닥에서 10% 정도 띄움
    left: "50%",
    transform: "translateX(-50%)", // 정중앙 정렬

    padding: "30px 70px",
    fontSize: "2rem", // 글자 아주 크게
    fontWeight: "bold",
    color: "white",
    backgroundColor: "#d9381e", // 타코 느낌의 칠리 레드
    border: "4px solid #f9c74f", // 치즈색 테두리
    borderRadius: "20px",
    cursor: "pointer",
    boxShadow: "0 10px 20px rgba(0,0,0,0.3)",
    transition: "transform 0.2s",
    animation: "pulse 1.5s infinite", // 깜빡이는 애니메이션
    zIndex: 10,
  },
};

// index.html이나 App.css에 넣어도 되지만, 편의상 스타일 태그 주입
const styleSheet = document.createElement("style");
styleSheet.innerText = `
  @keyframes pulse {
    0% { box-shadow: 0 0 0 0 rgba(217, 56, 30, 0.7); }
    70% { box-shadow: 0 0 0 20px rgba(217, 56, 30, 0); }
    100% { box-shadow: 0 0 0 0 rgba(217, 56, 30, 0); }
  }
`;
document.head.appendChild(styleSheet);
