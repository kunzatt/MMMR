package com.ssafy.mmmr.todos.dto;

import java.time.LocalDateTime;

import com.ssafy.mmmr.todos.entity.TodoEntity;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class TodoResponseDto {
	private Long id;
	private Long profileId;
	private String content;
	private Boolean isDone;
	private LocalDateTime createdAt;
	private LocalDateTime updatedAt;

	public static TodoResponseDto fromEntity(TodoEntity entity) {
		return TodoResponseDto.builder()
			.id(entity.getId())
			.profileId(entity.getProfile().getId())
			.content(entity.getContent())
			.isDone(entity.getIsDone())
			.createdAt(entity.getCreatedAt())
			.updatedAt(entity.getUpdatedAt())
			.build();
	}
}
