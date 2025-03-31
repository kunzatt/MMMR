package com.ssafy.mmmr.devices.service;

import java.util.List;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.devices.dto.HomeDeviceRequestDto;
import com.ssafy.mmmr.devices.dto.HomeDeviceResponseDto;
import com.ssafy.mmmr.devices.entity.HomeDeviceEntity;
import com.ssafy.mmmr.devices.repository.HomeDeviceRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.AccountException;
import com.ssafy.mmmr.global.error.exception.HomeDeviceException;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class HomeDeviceService {

	private final HomeDeviceRepository homeDeviceRepository;
	private final AccountRepository accountRepository;

	@Transactional(readOnly = true)
	public List<HomeDeviceResponseDto> findByAccountId(Long accountId) {
		AccountEntity account = accountRepository.findById(accountId)
			.orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));

		return homeDeviceRepository.findByAccount(account)
			.stream()
			.map(HomeDeviceResponseDto::new)
			.collect(Collectors.toList());
	}

	@Transactional
	public HomeDeviceResponseDto addDevice(HomeDeviceRequestDto requestDto) {
		AccountEntity account = accountRepository.findById(requestDto.getAccountId())
			.orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));

		HomeDeviceEntity homeDevice = HomeDeviceEntity.builder()
			.account(account)
			.device(requestDto.getDevice())
			.turned(requestDto.getTurned() != null ? requestDto.getTurned() : "OFF")
			.build();

		homeDeviceRepository.save(homeDevice);

		return new HomeDeviceResponseDto(homeDevice);
	}

	@Transactional
	public HomeDeviceResponseDto updateDevice(Long deviceId, Long accountId, String turned) {
		HomeDeviceEntity device = homeDeviceRepository.findById(deviceId)
			.orElseThrow(() -> new HomeDeviceException(ErrorCode.DEVICE_NOT_FOUND));

		if (!device.getAccount().getId().equals(accountId)) {
			throw new HomeDeviceException(ErrorCode.UNAUTHORIZED);
		}

		device.update(turned);

		return new HomeDeviceResponseDto(device);
	}

	@Transactional
	public void deleteDevice(Long deviceId, Long accountId) {
		HomeDeviceEntity device = homeDeviceRepository.findById(deviceId)
			.orElseThrow(() -> new HomeDeviceException(ErrorCode.DEVICE_NOT_FOUND));

		if (!device.getAccount().getId().equals(accountId)) {
			throw new HomeDeviceException(ErrorCode.UNAUTHORIZED);
		}

		homeDeviceRepository.delete(device);
	}
}