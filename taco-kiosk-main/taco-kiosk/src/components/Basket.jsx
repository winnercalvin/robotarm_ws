import React from "react";

export default function Basket({ basketItems, onOrder }) {
  return (
    <div className="basket-container">
      <h3>🧺 담은 재료</h3>
      <div style={{ minHeight: "60px" }}>
        {basketItems.length === 0 ? (
          <span style={{ color: "#aaa" }}>위에서 재료를 눌러보세요!</span>
        ) : (
          basketItems.map((item) => (
            <div key={item.uniqueId} className="dropped-item">
              {item.image ? (
                <img
                  src={item.image}
                  alt=""
                  style={{ width: "30px", verticalAlign: "middle" }}
                />
              ) : (
                item.emoji
              )}
              {" " + item.name}
            </div>
          ))
        )}
      </div>
      <button className="order-btn" onClick={onOrder}>
        주문하기
      </button>
    </div>
  );
}
