package dev.songm.taco.dtos;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
public class DrawPointDto {

    @JsonProperty("x")
    private String x; // JSON 값이 "0.253" 처럼 문자열로 오면 String, 숫자면 Double

    @JsonProperty("y")
    private String y;

    @JsonProperty("timestamp")
    private Long timestamp;

    // 편의 메서드: String으로 들어온 좌표를 숫자로 쓸 때 사용
    public double getXAsDouble() {
        return Double.parseDouble(this.x);
    }

    public double getYAsDouble() {
        return Double.parseDouble(this.y);
    }
}