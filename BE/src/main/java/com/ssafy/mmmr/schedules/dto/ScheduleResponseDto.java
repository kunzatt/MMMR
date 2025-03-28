package com.ssafy.mmmr.schedules.dto;

import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.mmmr.schedules.entity.ScheduleEntity;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ScheduleResponseDto {
	private Long id;
	private Long profileId;
	private String title;

	@JsonFormat(pattern = "yyyy-MM-dd HH:mm:ss")
	private LocalDateTime startDate;

	@JsonFormat(pattern = "yyyy-MM-dd HH:mm:ss")
	private LocalDateTime endDate;

	@JsonFormat(pattern = "yyyy-MM-dd HH:mm:ss")
	private LocalDateTime createdAt;

	@JsonFormat(pattern = "yyyy-MM-dd HH:mm:ss")
	private LocalDateTime updatedAt;

	public static ScheduleResponseDto of(ScheduleEntity schedule) {
		return ScheduleResponseDto.builder()
			.id(schedule.getId())
			.profileId(schedule.getProfile().getId())
			.title(schedule.getTitle())
			.startDate(schedule.getStartDate())
			.endDate(schedule.getEndDate())
			.createdAt(schedule.getCreatedAt())
			.updatedAt(schedule.getUpdatedAt())
			.build();
	}

	public static List<ScheduleResponseDto> of(List<ScheduleEntity> schedules) {
		return schedules.stream()
			.map(ScheduleResponseDto::of)
			.collect(Collectors.toList());
	}
}