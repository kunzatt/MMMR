package com.ssafy.mmmr.transportation.service;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestTemplate;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.TransportationException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.transportation.client.MetroClient;
import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.repository.BusRepository;
import com.ssafy.mmmr.transportation.repository.MetroInformationRepository;
import com.ssafy.mmmr.transportation.repository.MetroRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class TransportationService {

	private final MetroRepository metroRepository;
	private final BusRepository busRepository;
	private final ProfileRepository profileRepository;
	private final MetroInformationRepository metroInformationRepository;
	private final RestTemplate restTemplate;

	@Transactional
	public MetroEntity addMetro(MetroRequestDto requestDto) {
		ProfileEntity profile = profileRepository.findById(requestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (profile.getCount() > 3) {
			throw new TransportationException(ErrorCode.MORE_THAN_THREE_TRANSPORTATION);
		}

		MetroClient metroClient = new MetroClient(restTemplate);
		if (!metroClient.validateStation(requestDto.getStation())) {
			throw new TransportationException(ErrorCode.INVALID_STATION_NAME);
		}

		Integer lineNumber = metroClient.getLineNumber(requestDto.getStation());
		if (lineNumber == null) {
			throw new TransportationException(ErrorCode.INVALID_LINE_NUMBER);
		}

		MetroEntity metroEntity = MetroEntity.builder()
			.profile(profile)
			.line(lineNumber)
			.station(requestDto.getStation())
			.direction(requestDto.getDirection())
			.build();

		return metroRepository.save(metroEntity);
	}

	@Transactional
	public void deleteMetro(Long metroId, Long profileId) {
		MetroEntity metroEntity = metroRepository.findById(metroId)
			.orElseThrow(() -> new TransportationException(ErrorCode.METRO_NOT_FOUND));

		if (!metroEntity.getProfile().getId().equals(profileId)) {
			throw new ProfileException(ErrorCode.PROFILE_NOT_FOUND);
		}

		metroEntity.delete();
		metroRepository.save(metroEntity);
	}

}
