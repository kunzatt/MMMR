package com.ssafy.mmmr.devices.dto;

import com.ssafy.mmmr.devices.entity.HomeDeviceEntity;

import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class HomeDeviceResponseDto {

	private Long id;

	private Long accountId;

	private String device;

	private String turned;

	public HomeDeviceResponseDto(HomeDeviceEntity entity) {
		this.id = entity.getId();
		this.accountId = entity.getAccount().getId();
		this.device = entity.getDevice();
		this.turned = entity.getTurned();
	}
}
