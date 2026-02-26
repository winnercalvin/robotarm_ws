import React from "react";
import "../App.css"; // 스타일 가져오기

const TACO_PRICE = 4500; // 1세트당 가격

export default function OrderModal({ basket, onConfirm, onCancel }) {
  // 총 결제 금액 계산
  const totalAmount = basket.length * TACO_PRICE;

  return (
    <div className="modal-overlay">
      <div className="modal-content">
        <h2 style={{ borderBottom: "2px solid #333", paddingBottom: "10px" }}>
          🧾 주문 내역 확인
        </h2>

        {/* 스크롤 가능한 주문 내역 영역 */}
        <div
          className="order-summary"
          style={{ maxHeight: "300px", overflowY: "auto", textAlign: "left" }}
        >
          {basket.length === 0 ? (
            <p style={{ textAlign: "center", color: "#888" }}>
              담긴 메뉴가 없습니다.
            </p>
          ) : (
            <ul style={{ listStyle: "none", padding: 0 }}>
              {basket.map((taco, index) => (
                <li
                  key={taco.cartId}
                  className="summary-item"
                  style={{
                    borderTop: index === 0 ? "1px dashed #ccc" : "none",
                    borderBottom: "1px dashed #ccc",
                    padding: "15px 0",
                    display: "flex",
                    justifyContent: "space-between",
                    alignItems: "flex-start",
                  }}
                >
                  {/* 왼쪽: 메뉴 상세 정보 */}
                  <div>
                    <strong style={{ fontSize: "1.1rem", color: "#d9381e" }}>
                      🌮 타코 세트 {index + 1}
                    </strong>
                    <div
                      style={{
                        fontSize: "0.9rem",
                        color: "#555",
                        marginTop: "5px",
                        lineHeight: "1.6",
                      }}
                    >
                      • <strong>칩:</strong> {taco.chip.name}
                      <br />• <strong>소스:</strong> {taco.sauce.name}
                      {/* 그림 그린 경우 표시 */}
                      {taco.customPattern && (
                        <span
                          style={{
                            backgroundColor: "#ffe0e0",
                            color: "#d9381e",
                            fontSize: "0.8rem",
                            padding: "2px 6px",
                            borderRadius: "4px",
                            marginLeft: "5px",
                          }}
                        >
                          🎨 그림 요청
                        </span>
                      )}
                      <br />• <strong>토핑:</strong>{" "}
                      {taco.fillings.length > 0
                        ? taco.fillings.map((f) => f.name).join(", ")
                        : "없음"}
                    </div>
                  </div>

                  {/* 오른쪽: 가격 */}
                  <div style={{ fontWeight: "bold" }}>
                    {TACO_PRICE.toLocaleString()}원
                  </div>
                </li>
              ))}
            </ul>
          )}
        </div>

        {/* 하단 총 금액 표시 */}
        <div
          style={{
            borderTop: "2px solid #333",
            marginTop: "20px",
            paddingTop: "15px",
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            fontSize: "1.2rem",
            fontWeight: "bold",
          }}
        >
          <span>총 결제 금액</span>
          <span style={{ color: "#d9381e", fontSize: "1.5rem" }}>
            {totalAmount.toLocaleString()}원
          </span>
        </div>

        <div className="modal-actions" style={{ marginTop: "20px" }}>
          <button className="btn-cancel" onClick={onCancel}>
            취소
          </button>
          <button className="btn-confirm" onClick={onConfirm}>
            결제하기
          </button>
        </div>
      </div>
    </div>
  );
}
