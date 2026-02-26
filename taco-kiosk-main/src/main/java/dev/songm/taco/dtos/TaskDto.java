package dev.songm.taco.dtos;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.List;

@Data
@NoArgsConstructor
public class TaskDto {

    @JsonProperty("sequence")
    private int sequence;

    @JsonProperty("chip_id")
    private String chipId;

    @JsonProperty("topping_ids")
    private List<String> toppingIds;

    @JsonProperty("sauce_id")
    private String sauceId;

    @JsonProperty("draw_path")
    private List<DrawPointDto> drawPath; // 좌표 리스트

    @JsonProperty("is_custom")
    private boolean isCustom;
}