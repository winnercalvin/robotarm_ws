// src/components/TacoCard.jsx
import React from "react";

export default function TacoCard({ name, price, onOrder }) {
  return (
    <div
      style={{ border: "1px solid #ddd", padding: "20px", borderRadius: "8px" }}
    >
      {/* 타코 이름 표시 */}
      <h2>{name}</h2>

      {/* 가격 표시 */}
      <p>{price}원</p>

      {/* 주문 버튼: 클릭 시 부모에게 알림 */}
      <button onClick={onOrder} aria-label="주문하기">
        담기
      </button>
    </div>
  );
}
