package dev.songm.taco.entities;

import jakarta.persistence.*;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Entity
@Getter @Setter
@Table(name = "orders")
public class OrderEntity {
    @Id
    private Long orderId;

    private int totalPrice;
    private int taskCount;
    private LocalDateTime createdAt;
    @Column(nullable = false, length = 20)
    private String status = "COOKING";

    @OneToMany(mappedBy = "order", cascade = CascadeType.ALL)
    private List<TaskEntity> tasks = new ArrayList<>();

    public void addTask(TaskEntity task) {
        tasks.add(task);
        task.setOrder(this);
    }

    @PrePersist
    public void prePersist(){
        this.createdAt = LocalDateTime.now();
    }
}
