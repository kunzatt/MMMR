package com.ssafy.mmmr.schedules.dto;

import java.time.LocalDateTime;

import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class ScheduleRequestDto {

	private Long profileId;

	private String title;

	private LocalDateTime startDate;

	private LocalDateTime endDate;

}
