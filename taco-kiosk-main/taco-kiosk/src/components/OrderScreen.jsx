import axios from "axios";
import React, { useState } from "react";
import MenuSection from "./MenuSection";
import OrderModal from "./OrderModal";
import SauceCanvas from "./SauceCanvas"; // ⭐ 추가됨: 그리기 컴포넌트
import useVoiceOrder from "../hooks/useVoiceOrder";
import VoiceVisualizer from "./VoiceVisualizer";
import { SERVER_URL } from "../config";

const TACO_PRICE = 4500;

const INGREDIENTS = {
  chips: [
    {
      id: "basic",
      name: "감자칩 기본",
      image: "/images/potato.png",
    },
    {
      id: "double",
      name: "감자칩 두배",
      image: "/images/potato.png",
    },
  ],
  fillings: [
    { id: "cabbage", name: "양배추", image: "/images/cabbage.png" },
    { id: "tomato", name: "토마토", image: "/images/tomato.png" },
    { id: "onion", name: "양파", image: "/images/onion.png" },
  ],
  sauces: [
    { id: "tomato_sauce", name: "케챂", image: "/images/tomato_sauce.png" },
    { id: "mustard", name: "머스타드", image: "/images/mustard.png" },
  ],
};

export default function OrderScreen({ goHome }) {
  // 1. 현재 선택 중인 재료
  const [currentSelection, setCurrentSelection] = useState({
    chip: null,
    fillings: [],
    sauce: null,
  });

  // 2. 상태 관리
  const [cart, setCart] = useState([]);
  const [isModalOpen, setIsModalOpen] = useState(false);

  // ⭐ 소스 모달 상태 ('selection': 선택창, 'drawing': 그리기창, false: 닫힘)
  const [sauceMode, setSauceMode] = useState(false);

  // --- 핸들러: 재료 선택 ---
  const handleSelect = (item, type) => {
    if (type === "chip") {
      setCurrentSelection((prev) => ({ ...prev, chip: item }));
    } else if (type === "sauce") {
      setCurrentSelection((prev) => ({ ...prev, sauce: item }));
    } else if (type === "filling") {
      setCurrentSelection((prev) => {
        const exists = prev.fillings.find((f) => f.id === item.id);
        if (exists) {
          return {
            ...prev,
            fillings: prev.fillings.filter((f) => f.id !== item.id),
          };
        } else {
          return { ...prev, fillings: [...prev.fillings, item] };
        }
      });
    }
  };

  // --- ⭐ 추가된 음성 전용 핸들러: 모달 없이 즉시 담기 ---
  const handleVoiceAddToCart = () => {
    // 필수 항목 체크 (이미 음성 가이드에서 걸러지지만 안전을 위해 추가)
    if (!currentSelection.chip || !currentSelection.sauce) {
      console.log("❌ 필수 항목 미선택");
      return;
    }

    // ⭐ 핵심: 소스 패턴 선택 모달을 띄우지 않고 바로 null(기본 지그재그)을 넣어 담습니다.
    console.log("🎙️ 음성 인식 완료: 기본 패턴으로 장바구니 추가");
    finalAddToBasket(null);
  };

  // --- ⭐ 수정됨: [주문담기] 클릭 시 팝업 띄우기 ---
  const handleAddToCartClick = () => {
    // 유효성 검사
    if (!currentSelection.chip) {
      alert("감자칩 양을 선택해주세요!");
      return;
    }
    if (!currentSelection.sauce) {
      alert("소스를 선택해주세요!");
      return;
    }

    // 유효하면 소스 선택 모달 열기!
    setSauceMode("selection");
  };

  // --- ⭐ 추가됨: 최종 장바구니 담기 (패턴 데이터 포함) ---
  const finalAddToBasket = (patternData) => {
    const newTaco = {
      cartId: Date.now(),
      chip: currentSelection.chip,
      fillings: currentSelection.fillings,
      sauce: currentSelection.sauce,
      customPattern: patternData, // ⭐ 패턴 데이터 저장 (없으면 null)
      description: `${currentSelection.chip.name} + ${currentSelection.sauce.name}`,
    };

    setCart((prev) => [...prev, newTaco]);

    // 초기화 및 모달 닫기
    setCurrentSelection({ chip: null, fillings: [], sauce: null });
    setSauceMode(false);
  };

  // --- 핸들러: 장바구니 삭제 ---
  const removeFromCart = (cartId) => {
    setCart((prev) => prev.filter((item) => item.cartId !== cartId));
  };

  // --- 핸들러: [주문하기] (결제창 열기) ---
  const handlePlaceOrder = () => {
    if (cart.length === 0) {
      alert("장바구니가 비어있습니다.");
      return;
    }
    setIsModalOpen(true);
  };

  // --- 핸들러: 서버 전송 (로봇 데이터 포함) ---
  const handleFinalPayment = async () => {
    // 1. 로봇이 이해할 수 있는 형태로 데이터 변환 (매핑)
    const robotTasks = cart.map((taco, index) => ({
      sequence: index + 1, // 조리 순서 (1번째 타코, 2번째 타코...)

      // (1) 재료 ID 추출 (로봇 디스펜서 식별용)
      chip_id: taco.chip.id, // 예: 'basic' or 'double'
      topping_ids: taco.fillings.map((f) => f.id), // 예: ['onion', 'meat']
      sauce_id: taco.sauce.id, // 예: 'spicy'

      // (2) 소스 뿌리기 경로 (좌표 데이터)
      // customPattern이 있으면 그 좌표를 보내고, 없으면 null (로봇이 기본 패턴 사용)
      draw_path: taco.customPattern ? taco.customPattern : null,

      // (3) 메타 데이터 (옵션)
      is_custom: !!taco.customPattern, // 커스텀 여부 플래그
    }));

    // 2. 최종 전송 패킷 생성
    const orderData = {
      order_id: Date.now(), // 주문 고유 번호
      created_at: new Date().toISOString(), // 주문 시간
      total_price: cart.length * TACO_PRICE,
      task_count: robotTasks.length, // 총 만들어야 할 개수
      tasks: robotTasks, // 변환된 로봇 작업 리스트
    };

    try {
      // Python 서버로 전송 (IP주소 확인 필수!)
      const response = await axios.post(`${SERVER_URL}/api/order`, orderData);

      // 테스트용 로그 출력
      console.log("🤖 [로봇 전송 데이터 확인]");
      console.log(JSON.stringify(orderData, null, 2)); // 보기 좋게 출력

      alert("주문이 완료되었습니다! 로봇이 조리를 시작합니다. 🤖");

      // 상태 초기화 및 홈으로 이동
      setCart([]);
      goHome();
    } catch (e) {
      console.error("전송 에러:", e);
      alert("주문 전송 실패: 서버가 켜져 있는지 확인해주세요.");
    }
  };

  const { voiceStep } = useVoiceOrder(handleSelect, handleVoiceAddToCart);

  return (
    <div
      className="App"
      style={{
        minHeight: "100vh",
        paddingBottom: "200px",
        backgroundColor: "#f8f9fa",
      }}
    >
      {/* 상단 헤더 */}
      <div className="header-bar">
        <h2 style={{ margin: 0, fontSize: "1.5rem" }}>🌮 메뉴 선택</h2>
        <button className="btn-home" onClick={goHome} title="처음으로 돌아가기">
          <svg
            viewBox="0 0 24 24"
            fill="currentColor"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path d="M10 20v-6h4v6h5v-8h3L12 3 2 12h3v8z" />
          </svg>
        </button>
      </div>

      {/* 메뉴 선택 영역 */}
      <div style={{ padding: "20px" }}>
        <MenuSection
          title="Step 1. 감자칩 (필수)"
          items={INGREDIENTS.chips}
          selectedIds={currentSelection.chip?.id}
          onSelect={handleSelect}
          type="chip"
        />
        <MenuSection
          title="Step 2. 토핑 (중복 선택 가능)"
          items={INGREDIENTS.fillings}
          selectedIds={currentSelection.fillings.map((f) => f.id)}
          onSelect={handleSelect}
          type="filling"
        />
        <MenuSection
          title="Step 3. 소스 (필수)"
          items={INGREDIENTS.sauces}
          selectedIds={currentSelection.sauce?.id}
          onSelect={handleSelect}
          type="sauce"
        />
      </div>

      {/* 하단 고정 제어판 */}
      <div className="bottom-control-bar">
        {/* 왼쪽: 장바구니 리스트 */}
        <div className="cart-list-area">
          <h4 style={{ margin: "0 0 10px 0" }}>
            🛒 주문 목록 (Total: {cart.length}개 /{" "}
            {(cart.length * TACO_PRICE).toLocaleString()}원)
          </h4>
          {cart.length === 0 ? (
            <div
              style={{ color: "#aaa", marginTop: "20px", textAlign: "center" }}
            >
              메뉴를 선택하고 [주문담기]를 눌러주세요.
            </div>
          ) : (
            cart.map((taco, index) => (
              <div key={taco.cartId} className="cart-item">
                <div>
                  <div className="cart-item-title">
                    🌮 타코 세트 {index + 1}
                    <span
                      style={{
                        fontSize: "0.9rem",
                        color: "#888",
                        marginLeft: "5px",
                      }}
                    >
                      ({TACO_PRICE.toLocaleString()}원)
                    </span>
                  </div>
                  <div className="cart-item-detail">
                    {taco.chip.name} | {taco.sauce.name}
                    {taco.customPattern ? (
                      <span style={{ color: "#d9381e", fontWeight: "bold" }}>
                        {" "}
                        (🎨 그림 소스)
                      </span>
                    ) : (
                      ""
                    )}{" "}
                    <br />
                    (토핑:{" "}
                    {taco.fillings.map((f) => f.name).join(", ") || "없음"})
                  </div>
                </div>
                <button
                  className="btn-delete"
                  onClick={() => removeFromCart(taco.cartId)}
                >
                  삭제
                </button>
              </div>
            ))
          )}
        </div>

        {/* 오른쪽: 버튼 영역 */}
        <div className="action-buttons-area">
          {/* ⭐ 버튼 클릭 시 handleAddToCartClick 실행 */}
          <button className="btn-add-cart" onClick={handleAddToCartClick}>
            📥 주문담기
          </button>
          <button className="btn-place-order" onClick={handlePlaceOrder}>
            💳 주문하기
          </button>
        </div>
      </div>

      {/* ========================================= */}
      {/* ⭐ 1. 소스 뿌리기 방식 선택 모달 (중간 단계) */}
      {/* ========================================= */}
      {sauceMode === "selection" && (
        <div className="modal-overlay">
          <div className="modal-content" style={{ maxWidth: "400px" }}>
            <h3>🤖 로봇에게 알려주세요!</h3>
            <p>소스를 어떤 모양으로 뿌려드릴까요?</p>

            <div style={{ display: "flex", gap: "10px", marginTop: "20px" }}>
              {/* 옵션 A: 기본 (패턴 null) */}
              <button
                onClick={() => finalAddToBasket(null)}
                style={{
                  flex: 1,
                  padding: "15px",
                  borderRadius: "10px",
                  border: "2px solid #ddd",
                  background: "white",
                  cursor: "pointer",
                }}
              >
                <div style={{ fontSize: "2rem" }}>〰️</div>
                <strong>기본 지그재그</strong>
                <div style={{ fontSize: "0.8rem", color: "#888" }}>
                  알아서 뿌려주세요
                </div>
              </button>

              {/* 옵션 B: 그리기 (drawing 모드로 변경) */}
              <button
                onClick={() => setSauceMode("drawing")}
                style={{
                  flex: 1,
                  padding: "15px",
                  borderRadius: "10px",
                  border: "2px solid #d9381e",
                  background: "#fff0f0",
                  cursor: "pointer",
                }}
              >
                <div style={{ fontSize: "2rem" }}>🎨</div>
                <strong>직접 그리기</strong>
                <div style={{ fontSize: "0.8rem", color: "#d9381e" }}>
                  나만의 패턴 만들기
                </div>
              </button>
            </div>

            <button
              onClick={() => setSauceMode(false)}
              style={{
                marginTop: "20px",
                background: "transparent",
                border: "none",
                color: "#888",
                cursor: "pointer",
              }}
            >
              취소하고 다시 고르기
            </button>
          </div>
        </div>
      )}

      {/* ⭐ 2. 캔버스 모달 (직접 그리기 선택 시) */}
      {sauceMode === "drawing" && (
        <SauceCanvas
          onSave={(data) => finalAddToBasket(data)} // 저장하면 장바구니로 쏙!
          onClose={() => setSauceMode("selection")} // 닫으면 다시 선택창으로
        />
      )}

      {/* ⭐ 3. 최종 결제 확인 모달 */}
      {isModalOpen && (
        <OrderModal
          basket={cart}
          onConfirm={handleFinalPayment}
          onCancel={() => setIsModalOpen(false)}
        />
      )}
      <VoiceVisualizer status={voiceStep} />
    </div>
  );
}
