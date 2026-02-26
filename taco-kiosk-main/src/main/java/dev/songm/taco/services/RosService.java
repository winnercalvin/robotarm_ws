package dev.songm.taco.services;

import com.fasterxml.jackson.databind.ObjectMapper;
import dev.songm.taco.dtos.OrderRequest;
import dev.songm.taco.dtos.TaskDto;
import dev.songm.taco.entities.OrderEntity;
import dev.songm.taco.entities.TaskEntity;
import dev.songm.taco.repositories.OrderRepository;
import jakarta.annotation.PostConstruct;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketHttpHeaders;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.net.URI;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutionException;

@Service
public class RosService extends TextWebSocketHandler {
    private WebSocketSession session;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final OrderRepository orderRepository;
    private final SseService sseService;

    public RosService(OrderRepository orderRepository, SseService sseService){
        this.orderRepository = orderRepository;
        this.sseService = sseService;
    }

    @PostConstruct
    public void connectToRosBridge(){
        StandardWebSocketClient client = new StandardWebSocketClient();
        try{
            String rosUrl = "ws://192.168.10.75:9090";
            client.execute(this, new WebSocketHttpHeaders(), URI.create(rosUrl)).get();
        } catch (InterruptedException | ExecutionException e){
            System.err.println("연결 실패 : " + e.getMessage());
        }
    }
    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        this.session = session;
        System.out.println("✅ ROSBridge 연결 성공!");
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {
        System.out.println("❌ ROSBridge 연결 끊김");
        this.session = null;
    }

    public void sendOrderToRobot(OrderRequest orderRequest) {
        if (session != null && session.isOpen()) {
            try {
                String jsonOrder = objectMapper.writeValueAsString(orderRequest);

                Map<String, Object> rosMsg = new HashMap<>();
                rosMsg.put("op", "publish");
                rosMsg.put("topic", "/taco_order");
                rosMsg.put("type", "std_msgs/String");

                Map<String, String> innerMsg = new HashMap<>();
                innerMsg.put("data", jsonOrder);
                rosMsg.put("msg", innerMsg);

                session.sendMessage(new TextMessage(objectMapper.writeValueAsString(rosMsg)));
                System.out.println("🚀 로봇 전송 완료: " + jsonOrder);

            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            System.out.println("⚠️ 로봇 연결 안됨");
        }
    }

    public void saveOrder(OrderRequest dto) {
        // 1. Order Entity 생성
        OrderEntity order = new OrderEntity();
        order.setOrderId(dto.getOrderId());
        order.setTotalPrice(dto.getTotalPrice());
        order.setTaskCount(dto.getTaskCount());

        // 2. Task Entity 생성 및 매핑
        for (TaskDto taskDto : dto.getTasks()) {
            TaskEntity task = new TaskEntity();
            task.setSequence(taskDto.getSequence());
            task.setChipId(taskDto.getChipId());
            task.setSauceId(taskDto.getSauceId());
            task.setCustom(taskDto.isCustom());

            // 리스트 -> 문자열 변환 (예: ["A", "B"] -> "A,B")
            if (taskDto.getToppingIds() != null) {
                task.setToppings(String.join(",", taskDto.getToppingIds()));
            }

            // ⭐ 좌표 리스트 -> JSON 문자열로 변환 (Jackson 라이브러리 사용)
            if (taskDto.getDrawPath() != null) {
                try {
                    ObjectMapper mapper = new ObjectMapper();
                    String jsonString = mapper.writeValueAsString(taskDto.getDrawPath());
                    task.setDrawPathJson(jsonString);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            // 연관관계 설정
            order.addTask(task);
        }

        // 3. DB 저장 (Cascade 설정 때문에 order만 저장하면 task도 자동 저장됨)
        orderRepository.save(order);
        sseService.broadcastNewOrder(order);
    }
}
