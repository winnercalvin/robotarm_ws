package dev.songm.taco.controllers;

import dev.songm.taco.entities.OrderEntity;
import dev.songm.taco.repositories.OrderRepository;
import dev.songm.taco.services.SseService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;
import java.util.List;

@RestController
@RequestMapping("/api/orders")
@CrossOrigin(origins = "http://localhost:5173")
public class OrderController {

    private final OrderRepository orderRepository;
    private final SseService sseService; // 🌟 추가됨

    public OrderController(OrderRepository orderRepository, SseService sseService) {
        this.orderRepository = orderRepository;
        this.sseService = sseService;
    }

    // 기존: 처음에 켜질 때 전체 목록 1번 가져오기
    @GetMapping("/active")
    public List<OrderEntity> getActiveOrders() {
        return orderRepository.findAll();
    }

    // 🌟 추가됨: 리액트가 SSE 파이프를 연결하러 오는 주소
    @GetMapping(value = "/stream", produces = "text/event-stream")
    public SseEmitter stream() {
        return sseService.subscribe();
    }

    @PatchMapping("/{orderId}/status")
    public ResponseEntity<String> updateOrderStatus(
            @PathVariable Long orderId,
            @RequestParam String status) {

        // 1. DB에서 해당 주문 찾기 (없으면 에러 던짐)
        OrderEntity order = orderRepository.findById(orderId)
                .orElseThrow(() -> new IllegalArgumentException("주문을 찾을 수 없습니다: " + orderId));

        // 2. 상태를 파라미터로 넘어온 값(DONE)으로 변경하고 DB에 저장
        order.setStatus(status);
        orderRepository.save(order);

        // 3. 만약 다른 모니터(손님용 번호판 등)도 있다면 상태가 바뀌었다고 방송을 쏨!
        sseService.broadcastStatusUpdate(order);

        return ResponseEntity.ok("주문 " + orderId + " 상태 업데이트 완료: " + status);
    }
}