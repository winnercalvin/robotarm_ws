package dev.songm.taco.dtos;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.List;

@Data
@NoArgsConstructor
public class OrderRequest {

    @JsonProperty("order_id")
    private Long orderId;

    @JsonProperty("created_at")
    private String createAt;

    @JsonProperty("total_price")
    private int totalPrice;

    @JsonProperty("task_count")
    private int taskCount;

    @JsonProperty("tasks")
    private List<TaskDto> tasks;
}