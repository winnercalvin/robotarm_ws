import { useState, useEffect, useRef, useCallback } from "react";

export default function useVoiceOrder(handleSelect, handleAddToCartClick) {
  const [voiceStep, setVoiceStep] = useState("IDLE");
  const stepRef = useRef("IDLE");
  const recognitionRef = useRef(null);
  const silenceTimer = useRef(null);

  // 🔥 핵심: 최신 함수를 참조하기 위한 Ref 사용
  const selectRef = useRef(handleSelect);
  const addToCartRef = useRef(handleAddToCartClick);

  useEffect(() => {
    selectRef.current = handleSelect;
    addToCartRef.current = handleAddToCartClick;
  }, [handleSelect, handleAddToCartClick]);

  const speak = (text) => {
    if (!window.speechSynthesis) return;
    window.speechSynthesis.cancel();
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.lang = "ko-KR";
    utterance.rate = 1.0;
    window.speechSynthesis.speak(utterance);
  };

  const resetSilenceTimer = () => {
    if (silenceTimer.current) clearTimeout(silenceTimer.current);
    if (stepRef.current !== "IDLE") {
      silenceTimer.current = setTimeout(() => remindUser(), 10000);
    }
  };

  const remindUser = () => {
    const step = stepRef.current;
    if (step === "STEP1") speak("아직 못 정하셨나요? 기본과 두배 중에 말씀해 주세요.");
    else if (step === "STEP2") speak("토핑을 골라주세요. 양파, 토마토, 양배추가 있습니다.");
    else if (step === "STEP3") speak("소스는 케챂과 머스타드가 있습니다.");
  };

  const changeStep = (nextStep) => {
    stepRef.current = nextStep;
    setVoiceStep(nextStep);
    resetSilenceTimer();
  };

  // 🧠 명령 처리 함수 (함수 선언 순서가 중요할 수 있어 내부로 이동)
  const processCommand = (command) => {
    const currentStep = stepRef.current;

    if (command.includes("처음") || command.includes("취소") || command.includes("리셋")) {
      speak("네, 처음으로 돌아갑니다.");
      changeStep("IDLE");
      return;
    }

    if (currentStep === "IDLE") {
      if (command.includes("헤이") || command.includes("타코") || command.includes("주문")) {
        changeStep("STEP1");
        speak("네! 감자칩 양을 골라주세요. 기본과 두배가 있습니다.");
      }
      return;
    }

    if (currentStep === "STEP1") {
      if (command.includes("기본")) {
        selectRef.current({ id: "basic", name: "감자칩 기본" }, "chip"); // Ref 사용
        goToNextStep("STEP2", "기본입니다. 토핑은요? 양파, 토마토, 양배추가 있습니다.");
      } else if (command.includes("두배") || command.includes("두 배")) {
        selectRef.current({ id: "double", name: "감자칩 두배" }, "chip"); // Ref 사용
        goToNextStep("STEP2", "두배입니다. 토핑은요? 양파, 토마토, 양배추가 있습니다.");
      }
      return;
    }

    if (currentStep === "STEP2") {
      if (command.includes("양파")) selectRef.current({ id: "onion", name: "양파" }, "filling");
      if (command.includes("토마토")) selectRef.current({ id: "tomato", name: "토마토" }, "filling");
      if (command.includes("양배추")) selectRef.current({ id: "cabbage", name: "양배추" }, "filling");

      if (command.includes("다음") || command.includes("끝") || command.includes("없어")) {
        goToNextStep("STEP3", "소스는 뭘로 할까요? 케챂과 머스타드가 있습니다.");
      }
      return;
    }

    if (currentStep === "STEP3") {
      if (command.includes("케챂") || command.includes("케찹")) {
        selectRef.current({ id: "tomato_sauce", name: "케챂" }, "sauce");
        finishOrder("케챂으로 담아드릴게요.");
      } else if (command.includes("머스") || command.includes("머스타드")) {
        selectRef.current({ id: "mustard", name: "머스타드" }, "sauce");
        finishOrder("머스타드로 담아드릴게요.");
      }
    }
  };

  const goToNextStep = (nextStep, message) => {
    changeStep(nextStep);
    speak(message);
  };

  const finishOrder = (message) => {
    speak(message);
    // ✅ 장바구니 담기 실행 전 딜레이를 주되, 최신 Ref를 호출
    setTimeout(() => {
      console.log("🛒 장바구니 담기 실행!");
      addToCartRef.current(); 
      changeStep("IDLE");
    }, 1500);
  };

  useEffect(() => {
    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRecognition) return;

    const recognition = new SpeechRecognition();
    recognition.lang = "ko-KR";
    recognition.continuous = true;
    recognition.interimResults = false;

    recognition.onresult = (event) => {
      const lastResult = event.results[event.results.length - 1];
      const command = lastResult[0].transcript.trim();
      resetSilenceTimer();
      processCommand(command);
    };

    recognition.onend = () => {
      // ⚠️ 수동 중단이 아닐 때만 다시 시작
      if (stepRef.current !== "FINISHED") {
        recognition.start();
      }
    };

    recognitionRef.current = recognition;
    recognition.start();

    return () => {
      if (recognitionRef.current) recognitionRef.current.stop();
      if (silenceTimer.current) clearTimeout(silenceTimer.current);
    };
  }, []);

  return { voiceStep };
}