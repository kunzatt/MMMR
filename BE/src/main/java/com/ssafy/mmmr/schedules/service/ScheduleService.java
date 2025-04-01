package com.ssafy.mmmr.schedules.service;

import java.util.List;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.ScheduleException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.schedules.dto.ScheduleRequestDto;
import com.ssafy.mmmr.schedules.dto.ScheduleResponseDto;
import com.ssafy.mmmr.schedules.entity.ScheduleEntity;
import com.ssafy.mmmr.schedules.repository.ScheduleRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class ScheduleService {

	private final ScheduleRepository scheduleRepository;
	private final ProfileRepository profileRepository;

	@Transactional(readOnly = true)
	public ScheduleResponseDto getScheduleById(Long id, AuthUser authUser) {
		ScheduleEntity schedule = scheduleRepository.findByIdAndNotDeleted(id)
			.orElseThrow(() -> new ScheduleException(ErrorCode.SCHEDULE_NOT_FOUND));

		if (!schedule.getProfile().getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		return ScheduleResponseDto.of(schedule);
	}

	@Transactional
	public ScheduleResponseDto addSchedule(ScheduleRequestDto requestDto, AuthUser authUser) {
		if (requestDto.getStartDate().isAfter(requestDto.getEndDate())) {
			throw new ScheduleException(ErrorCode.INVALID_END_DATE);
		}

		ProfileEntity profile = profileRepository.findById(requestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (!profile.getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		ScheduleEntity schedule = ScheduleEntity.builder()
			.profile(profile)
			.title(requestDto.getTitle())
			.startDate(requestDto.getStartDate())
			.endDate(requestDto.getEndDate())
			.build();

		ScheduleEntity savedSchedule = scheduleRepository.save(schedule);
		return ScheduleResponseDto.of(savedSchedule);
	}

	@Transactional(readOnly = true)
	public List<ScheduleResponseDto> getSchedulesByProfile(Long profileId, AuthUser authUser) {
		ProfileEntity profile = profileRepository.findById(profileId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (!profile.getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		List<ScheduleEntity> schedules = scheduleRepository.findByProfileIdAndNotDeleted(profileId);
		return ScheduleResponseDto.of(schedules);
	}

	@Transactional(readOnly = true)
	public List<ScheduleResponseDto> searchSchedulesByProfileAndKeyword(Long profileId, String keyword, AuthUser authUser) {
		ProfileEntity profile = profileRepository.findById(profileId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (!profile.getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		List<ScheduleEntity> schedules = scheduleRepository.searchByProfileIdAndKeyword(profileId, keyword);
		return ScheduleResponseDto.of(schedules);
	}

	@Transactional
	public ScheduleResponseDto updateSchedule(Long id, ScheduleRequestDto requestDto, AuthUser authUser) {
		ScheduleEntity schedule = scheduleRepository.findByIdAndNotDeleted(id)
			.orElseThrow(() -> new ScheduleException(ErrorCode.SCHEDULE_NOT_FOUND));

		if (!schedule.getProfile().getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		schedule.update(requestDto.getTitle(), requestDto.getStartDate(), requestDto.getEndDate());

		if (schedule.getStartDate() != null && schedule.getEndDate() != null) {
			if (schedule.getStartDate().isAfter(schedule.getEndDate())) {
				throw new ScheduleException(ErrorCode.INVALID_END_DATE);
			}
		}

		ScheduleEntity updatedSchedule = scheduleRepository.save(schedule);
		return ScheduleResponseDto.of(updatedSchedule);
	}

	@Transactional
	public void deleteSchedule(Long id, AuthUser authUser) {
		ScheduleEntity schedule = scheduleRepository.findByIdAndNotDeleted(id)
			.orElseThrow(() -> new ScheduleException(ErrorCode.SCHEDULE_NOT_FOUND));

		if (!schedule.getProfile().getAccount().getId().equals(authUser.getId())) {
			throw new ScheduleException(ErrorCode.UNAUTHORIZED);
		}

		schedule.delete();
		scheduleRepository.save(schedule);
	}
}