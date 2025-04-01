package com.ssafy.mmmr.profiles.dto;

import java.time.LocalDateTime;

import com.ssafy.mmmr.profiles.entity.CallSign;

import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Builder
public class ProfileResponseDto {

	private Long id;

	private String nickname;

	private CallSign callSign;

}
