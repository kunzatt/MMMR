package com.ssafy.mmmr.devices.dto;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
public class HomeDeviceRequestDto {

	private Long accountId;

	private String device;

	private String turned;

}
