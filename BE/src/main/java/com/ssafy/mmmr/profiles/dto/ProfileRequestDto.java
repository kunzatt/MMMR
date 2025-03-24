package com.ssafy.mmmr.profiles.dto;

import com.ssafy.mmmr.profiles.entity.CallSign;

import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ProfileRequestDto {

	private String nickname;

	private CallSign callSign;

}
