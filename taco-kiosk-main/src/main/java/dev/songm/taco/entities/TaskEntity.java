package dev.songm.taco.entities;

import jakarta.persistence.*;
import lombok.Getter;
import lombok.Setter;

@Entity
@Getter @Setter
@Table(name = "tasks")
public class TaskEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id; // 작업 고유 번호

    private int sequence;
    private String chipId;
    private String sauceId;

    // 토핑 리스트는 "tomato,onion" 처럼 문자열로 합쳐서 저장 추천
    private String toppings;

    // ⭐ 핵심: 수백 개의 좌표 데이터는 아주 긴 텍스트(JSON String)로 저장
    @Lob // 대용량 텍스트 저장용
    @Column(columnDefinition = "TEXT")
    private String drawPathJson;

    private boolean isCustom;

    @ManyToOne
    @JoinColumn(name = "order_id")
    @com.fasterxml.jackson.annotation.JsonIgnore
    private OrderEntity order;
}