import React, { useState, useEffect } from "react";
import AdminDashboardDesktop from "./AdminDashboardDesktop"; // 기존 원본 파일
import AdminDashboardMobile from "./AdminDashboardMobile";   // 방금 만든 모바일 전용 파일

export default function AdminDashboard({ onBack }) {
  // 화면 가로 길이가 768px 이하인지 체크합니다.
  const [isMobile, setIsMobile] = useState(window.innerWidth <= 768);

  useEffect(() => {
    // 창 크기가 변할 때마다 감지해서 렌더링을 바꿉니다.
    const handleResize = () => setIsMobile(window.innerWidth <= 768);
    window.addEventListener("resize", handleResize);
    
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  // 모바일 화면이면 모바일 컴포넌트를, 아니면 데스크탑 원본을 보여줍니다.
  if (isMobile) {
    return <AdminDashboardMobile onBack={onBack} />;
  }
  
  return <AdminDashboardDesktop onBack={onBack} />;
}