import React from "react";

// selectedIds: 이제 문자열 하나가 아니라, 배열[]도 받을 수 있게 처리함
export default function MenuSection({
  title,
  items,
  selectedIds,
  onSelect,
  type,
}) {
  // ⭐ 선택 여부 확인 함수 (단일 값 or 배열 모두 처리)
  const isSelected = (id) => {
    if (Array.isArray(selectedIds)) {
      return selectedIds.includes(id); // 배열에 포함되어 있나? (토핑용)
    }
    return selectedIds === id; // 값이 같은가? (감자칩/소스용)
  };

  const cardTypeClass = type === "chip" ? "transparent-card" : "";

  return (
    <div style={{ textAlign: "left", marginBottom: "30px" }}>
      <div className="section-title" style={{ fontSize: "1.5rem" }}>
        {title}
      </div>
      <div>
        {items.map((item) => {
          const active = isSelected(item.id); // 선택 여부 저장

          return (
            <div
              key={item.id}
              // 선택되면 'selected' 클래스 추가 -> CSS 효과 발동
              className={`option-card ${active ? "selected" : ""} ${cardTypeClass}`}
              onClick={() => onSelect(item, type)}
              style={{
                position: "relative",
                width: type === "chip" ? "180px" : "180px",
                height: type === "chip" ? "210px" : "210px",
                margin: "15px",
              }}
            >
              {item.id === "double" && <div className="badge-2x">2x</div>}
              {/* ✅ 선택되면 체크 표시 띄우기 */}
              {active && <div className="check-badge">✔</div>}

              {item.image ? (
                <img
                  src={item.image}
                  alt={item.name}
                  className="card-img"
                  style={{
                    width: "100%",
                    height: "140px",
                    objectFit: "contain",
                  }}
                />
              ) : (
                <div style={{ fontSize: "60px", lineHeight: "1.2" }}>
                  {item.emoji}
                </div>
              )}

              <p
                style={{
                  fontWeight: "bold",
                  margin: "0",
                  fontSize: "1.2rem",
                  textShadow:
                    type === "chip"
                      ? "1px 1px 2px rgba(255,255,255,0.8)"
                      : "none",
                }}
              >
                {item.name}
              </p>
            </div>
          );
        })}
      </div>
    </div>
  );
}
