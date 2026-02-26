package dev.songm.taco.services;

import org.springframework.stereotype.Service;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

@Service
public class SseService {
    // 현재 모니터링 화면(리액트)을 켜둔 사용자들의 파이프 목록
    private final List<SseEmitter> emitters = new CopyOnWriteArrayList<>();

    // 1. 리액트가 처음 접속할 때 파이프(Emitter)를 연결해주는 메서드
    public SseEmitter subscribe() {
        SseEmitter emitter = new SseEmitter(Long.MAX_VALUE); // 타임아웃 무한대
        emitters.add(emitter);

        // 클라이언트가 브라우저를 끄거나 연결이 끊기면 리스트에서 제거
        emitter.onCompletion(() -> emitters.remove(emitter));
        emitter.onTimeout(() -> emitters.remove(emitter));
        emitter.onError((e) -> emitters.remove(emitter));

        try {
            // 처음 연결되었을 때 더미 데이터를 보내서 연결 성공을 알림 (503 에러 방지)
            emitter.send(SseEmitter.event().name("connect").data("SSE Connected!"));
        } catch (IOException e) {
            emitters.remove(emitter);
        }

        return emitter;
    }

    // 2. 새로운 주문이 들어왔을 때 연결된 모든 리액트 화면에 데이터를 쏘는 메서드
    public void broadcastNewOrder(Object orderData) {
        for (SseEmitter emitter : emitters) {
            try {
                // "newOrder" 라는 이름표를 붙여서 데이터를 쏩니다.
                emitter.send(SseEmitter.event().name("newOrder").data(orderData));
            } catch (IOException e) {
                emitters.remove(emitter);
            }
        }
    }

    public void broadcastStatusUpdate(Object orderData) {
        for (SseEmitter emitter : emitters) {
            try {
                // "statusUpdate" 라는 이름표를 달아서 보냅니다.
                emitter.send(SseEmitter.event().name("statusUpdate").data(orderData));
            } catch (IOException e) {
                emitters.remove(emitter);
            }
        }
    }
}