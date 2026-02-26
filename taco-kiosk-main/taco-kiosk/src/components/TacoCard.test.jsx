// src/components/TacoCard.test.jsx
import { render, screen, fireEvent } from "@testing-library/react";
import { describe, it, expect, vi } from "vitest";
import TacoCard from "./TacoCard";

describe("TacoCard 컴포넌트 테스트", () => {
  // 1. 렌더링 테스트: 화면에 글자가 잘 나오나?
  it("타코 이름과 가격이 올바르게 표시되어야 한다", () => {
    render(<TacoCard name="불고기 타코" price={5000} />);

    // 화면에 '불고기 타코'라는 글자가 있는지 확인
    expect(screen.getByText("불고기 타코")).toBeInTheDocument();
    // 화면에 '5000원'이라는 글자가 있는지 확인
    expect(screen.getByText("5000원")).toBeInTheDocument();
  });

  // 2. 기능 테스트: 버튼을 누르면 주문 함수가 실행되나?
  it("담기 버튼을 클릭하면 onOrder 함수가 호출되어야 한다", () => {
    // 가짜 함수(Mock Function) 생성
    const mockOnOrder = vi.fn();

    render(<TacoCard name="치킨 타코" price={4500} onOrder={mockOnOrder} />);

    // '주문하기' 버튼을 찾아서 클릭
    const button = screen.getByLabelText("주문하기");
    fireEvent.click(button);

    // 가짜 함수가 1번 호출되었는지 확인
    expect(mockOnOrder).toHaveBeenCalledTimes(1);
  });
});
