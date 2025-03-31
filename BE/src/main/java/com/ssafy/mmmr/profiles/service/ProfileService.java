package com.ssafy.mmmr.profiles.service;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.AccountException;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.profiles.dto.CallSignResponseDto;
import com.ssafy.mmmr.profiles.dto.ProfileRequestDto;
import com.ssafy.mmmr.profiles.dto.ProfileResponseDto;
import com.ssafy.mmmr.profiles.dto.ProfileUpdateRequestDto;
import com.ssafy.mmmr.profiles.entity.CallSign;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class ProfileService {

	private final ProfileRepository profileRepository;
	private final AccountRepository accountRepository;

	@Transactional
	public Long addProfile(Long accountId, ProfileRequestDto profileRequestDto) {
		AccountEntity account = accountRepository.findById(accountId)
			.orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));

		if (profileRepository.existsByNicknameAndDeletedFalse(profileRequestDto.getNickname())) {
			throw new ProfileException(ErrorCode.NICKNAME_EXISTS);
		}

		boolean callSignInUse = profileRepository.findByAccountIdAndDeletedFalse(accountId)
			.stream()
			.anyMatch(profile -> profile.getCallSign() == profileRequestDto.getCallSign());

		if (callSignInUse) {
			throw new ProfileException(ErrorCode.CALLSIGN_EXISTS);
		}

		ProfileEntity profile = ProfileEntity.builder()
			.account(account)
			.nickname(profileRequestDto.getNickname())
			.callSign(profileRequestDto.getCallSign())
			.build();

		account.addProfile(profile);

		ProfileEntity savedProfile = profileRepository.save(profile);

		return savedProfile.getId();
	}

	@Transactional(readOnly = true)
	public List<ProfileResponseDto> getProfileList(Long accountId) {
		accountRepository.findById(accountId).orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));
		List<ProfileEntity> profiles = profileRepository.findByAccountIdAndDeletedFalse(accountId);

		return profiles.stream()
			.map(this::profile)
			.collect(Collectors.toList());
	}

	@Transactional
	public void updateProfile(Long profileId, Long accountId, ProfileUpdateRequestDto updateDto) {
		ProfileEntity profile = profileRepository.findByIdAndAccountIdAndDeletedFalse(profileId, accountId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (updateDto.getNickname() != null && !updateDto.getNickname().equals(profile.getNickname())) {
			if (profileRepository.existsByNicknameAndDeletedFalse(updateDto.getNickname())) {
				throw new ProfileException(ErrorCode.NICKNAME_EXISTS);
			}
			profile.changeNickname(updateDto.getNickname());
		}

		if (updateDto.getCallSign() != null && !updateDto.getCallSign().equals(profile.getCallSign())) {
			boolean callSignInUse = profileRepository.findByAccountIdAndDeletedFalse(accountId)
				.stream()
				.filter(p -> !p.getId().equals(profileId))
				.anyMatch(p -> p.getCallSign() == updateDto.getCallSign());

			if (callSignInUse) {
				throw new ProfileException(ErrorCode.CALLSIGN_EXISTS);
			}

			profile.changeCallSign(updateDto.getCallSign());
		}
	}

	@Transactional
	public void deleteProfile(Long profileId, Long accountId) {
		ProfileEntity profile = profileRepository.findByIdAndAccountIdAndDeletedFalse(profileId, accountId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		profile.delete();

		profileRepository.save(profile);

		profileRepository.flush();
	}

	@Transactional(readOnly = true)
	public List<CallSignResponseDto> getAvailableCallSigns(Long accountId) {
		List<CallSign> allCallSigns = Arrays.asList(CallSign.values());
		List<CallSign> usedCallSigns = profileRepository.findByAccountIdAndDeletedFalse(accountId)
			.stream()
			.map(ProfileEntity::getCallSign)
			.collect(Collectors.toList());

		return allCallSigns.stream()
			.filter(callSign -> !usedCallSigns.contains(callSign))
			.map(CallSignResponseDto::from)
			.collect(Collectors.toList());
	}

	private ProfileResponseDto profile(ProfileEntity profile) {
		return ProfileResponseDto.builder()
			.id(profile.getId())
			.nickname(profile.getNickname())
			.callSign(profile.getCallSign())
			.build();
	}
}