package dev.songm.taco.controllers;

import dev.songm.taco.dtos.OrderRequest;
import dev.songm.taco.entities.OrderEntity;
import dev.songm.taco.repositories.OrderRepository;
import dev.songm.taco.services.RosService;
import org.springframework.data.domain.Sort;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
public class ProductController {
    private final RosService rosService;
    private final OrderRepository orderRepository;

    public ProductController(RosService rosService, OrderRepository orderRepository){
        this.orderRepository = orderRepository;
        this.rosService = rosService;
    }

    @PostMapping("/api/order")
    public String placeOrder(@RequestBody OrderRequest orderRequest){
        System.out.println("주문 번호: " + orderRequest.getOrderId());

        // 데이터 접근 예시
        if (orderRequest.getTasks() != null) {
            for (var task : orderRequest.getTasks()) {
                System.out.println("소스 종류: " + task.getSauceId());

                if (task.getDrawPath() != null) {
                    System.out.println("좌표 개수: " + task.getDrawPath().size());
                }
            }
        }
        rosService.saveOrder(orderRequest);
        rosService.sendOrderToRobot(orderRequest);
        return "주문 접수 완료";
    }

    @GetMapping("/api/admin/orders")
    public List<OrderEntity> getAllOrdersForAdmin() {
        // ID 내림차순(최신순)으로 정렬해서 가져오기
        return orderRepository.findAll(Sort.by(Sort.Direction.DESC, "orderId"));
    }

    @DeleteMapping("/api/admin/orders")
    public String deleteAllOrders() {
        orderRepository.deleteAll();
        return "데이터가 모두 삭제되었습니다.";
    }
}
