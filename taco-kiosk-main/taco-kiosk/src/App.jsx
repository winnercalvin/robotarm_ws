import React, { useState } from "react";
import { BrowserRouter, Routes, Route, useNavigate } from "react-router-dom";
import StartButton from "./components/StartButton";
import OrderScreen from "./components/OrderScreen";
import AdminDashboard from "./dashboards/AdminDashboard"; // 경로 확인 필요 (screens 폴더)
import OrderMonitor from "./components/OrderMonitor"; 
import "./App.css";

function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Routes>
          {/* 1. 관리자 페이지 (주소창에 /admin 입력 시에만 접속) */}
          <Route path="/admin" element={<AdminPageWrapper />} />

          <Route path="/monitor" element={<OrderMonitor />} />

          {/* 2. 기본 키오스크 화면 (대기화면 <-> 주문화면) */}
          <Route path="/" element={<KioskMain />} />
        </Routes>
      </div>
    </BrowserRouter>
  );
}

// --- [서브 컴포넌트 1] 관리자 페이지 래퍼 ---
function AdminPageWrapper() {
  const navigate = useNavigate();
  // 관리자 화면에서 '홈으로' 버튼 누르면 메인으로 이동
  return <AdminDashboard onBack={() => navigate("/")} />;
}

// --- [서브 컴포넌트 2] 일반 키오스크 로직 (기존 코드) ---
function KioskMain() {
  const [isOrderStarted, setIsOrderStarted] = useState(false);

  const startOrder = () => setIsOrderStarted(true);

  const goHome = () => {
    setIsOrderStarted(false);
  };

  // A. 대기 화면 (포스터 + 시작 버튼)
  if (!isOrderStarted) {
    return (
      <div
        style={{
          width: "100vw",
          height: "100vh",
          backgroundImage: "url(/images/main_image.png)",
          backgroundSize: "100% 100%",
          backgroundPosition: "center",
          backgroundRepeat: "no-repeat",
          position: "relative",
        }}
      >
        <StartButton onClick={startOrder} />
      </div>
    );
  }

  // B. 주문 화면
  return <OrderScreen goHome={goHome} />;
}

export default App;
