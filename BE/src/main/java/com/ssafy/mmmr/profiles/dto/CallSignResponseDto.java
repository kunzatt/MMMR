package com.ssafy.mmmr.profiles.dto;

import com.ssafy.mmmr.profiles.entity.CallSign;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class CallSignResponseDto {

	private String name;

	public static CallSignResponseDto from(CallSign callSign) {
		return CallSignResponseDto.builder()
			.name(callSign.name())
			.build();
	}
}