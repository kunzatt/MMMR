package com.ssafy.mmmr.transportation.service;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestTemplate;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.repository.BusInformationRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.TransportationException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.transportation.client.MetroClient;
import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
import com.ssafy.mmmr.transportation.dto.TransportationSearchRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationSearchResponseDto;
import com.ssafy.mmmr.transportation.entity.BusEntity;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.entity.MetroInformationEntity;
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
	private final BusInformationRepository busInformationRepository;
	private final RestTemplate restTemplate;

	public List<TransportationSearchResponseDto> searchTransportation(String type, String keyword) {
		List<TransportationSearchResponseDto> results = new ArrayList<>();

		// type이 ALL 또는 BUS인 경우 버스 정보 검색
		if ("ALL".equalsIgnoreCase(type) || "BUS".equalsIgnoreCase(type)) {
			List<BusInformationEntity> busResults = busInformationRepository.searchByKeyword(keyword);

			results.addAll(busResults.stream()
				.map(bus -> TransportationSearchResponseDto.builder()
					.type("BUS")
					.station(bus.getStation())
					.sequence(String.valueOf(bus.getSequence()))
					.number(bus.getRoute())
					.information("정류장ID: " + bus.getStationId() + ", 노선ID: " + bus.getRouteId())
					.build())
				.collect(Collectors.toList()));
		}

		// type이 ALL 또는 METRO인 경우 지하철 정보 검색
		if ("ALL".equalsIgnoreCase(type) || "METRO".equalsIgnoreCase(type)) {
			List<MetroInformationEntity> metroResults = metroInformationRepository.searchByKeyword(keyword);

			results.addAll(metroResults.stream()
				.map(metro -> TransportationSearchResponseDto.builder()
					.type("METRO")
					.station(metro.getStationName())
					.sequence("")
					.number(metro.getLineNumber())
					.information("지하철 " + metro.getLineNumber())
					.build())
				.collect(Collectors.toList()));
		}

		return results;
	}

	@Transactional
	public Object addTransportation(TransportationRequestDto requestDto, AuthUser authUser) {
		// 1. 요청된 프로필 조회
		ProfileEntity profile = profileRepository.findById(requestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		// 2. 해당 프로필이 현재 사용자의 계정에 속하는지 검증
		if (!profile.getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		// 3. 프로필당 최대 3개의 대중교통 정보만 허용
		if (profile.getCount() >= 3) {
			throw new TransportationException(ErrorCode.MORE_THAN_THREE_TRANSPORTATION);
		}

		// 4. 타입에 따라 버스 또는 지하철 정보 추가
		if ("BUS".equalsIgnoreCase(requestDto.getType())) {
			return addBus(profile, requestDto);
		} else if ("METRO".equalsIgnoreCase(requestDto.getType())) {
			return addMetro(profile, requestDto);
		} else {
			throw new TransportationException(ErrorCode.INVALID_TRANSPORTATION_TYPE);
		}
	}

	private BusEntity addBus(ProfileEntity profile, TransportationRequestDto requestDto) {
		// 버스 정보 유효성 검사
		if (requestDto.getNumber() == null || requestDto.getStation() == null ||
			requestDto.getRouteId() == null || requestDto.getStationId() == null) {
			throw new TransportationException(ErrorCode.INVALID_BUS_INFORMATION);
		}

		// 현재 버스 정보 조회 - routeId와 stationId로 명확하게 식별
		BusInformationEntity currentBusInfo = busInformationRepository
			.findByRouteIdAndStationId(
				requestDto.getRouteId(),
				requestDto.getStationId())
			.orElseThrow(() -> new TransportationException(ErrorCode.INVALID_BUS_INFORMATION));

		// 다음 정거장 정보 조회 - 현재 정거장의 sequence + 1인 정거장 찾기
		BusInformationEntity nextBusInfo = busInformationRepository
			.findByRouteIdAndSequence(
				requestDto.getRouteId(),
				currentBusInfo.getSequence() + 1)
			.orElse(null);

		// direction 자동 설정
		String direction = nextBusInfo != null ? nextBusInfo.getStation() : "종점";

		BusEntity busEntity = BusEntity.builder()
			.profile(profile)
			.routeId(requestDto.getRouteId())
			.route(requestDto.getNumber())
			.stationId(requestDto.getStationId())
			.station(requestDto.getStation())
			.direction(direction)
			.build();

		return busRepository.save(busEntity);
	}

	private MetroEntity addMetro(ProfileEntity profile, TransportationRequestDto requestDto) {
		if (requestDto.getNumber() == null || requestDto.getStation() == null) {
			throw new TransportationException(ErrorCode.INVALID_METRO_INFORMATION);
		}

		// QueryDsl을 사용한 유연한 검색
		MetroInformationEntity metroInfo = metroInformationRepository
			.findByFlexibleLineNumberAndStationName(requestDto.getNumber(), requestDto.getStation())
			.orElseThrow(() -> new TransportationException(ErrorCode.INVALID_STATION_NAME));

		Integer lineNumber = Integer.parseInt(
			metroInfo.getLineNumber().replaceAll("[^0-9]", "")
		);

		MetroEntity metroEntity = MetroEntity.builder()
			.profile(profile)
			.line(lineNumber)
			.station(requestDto.getStation())
			.build();

		return metroRepository.save(metroEntity);
	}

	@Transactional
	public void deleteTransportation(Long transportationId, String type, AuthUser authUser) {
		if ("BUS".equalsIgnoreCase(type)) {
			deleteBus(transportationId, authUser);
		} else if ("METRO".equalsIgnoreCase(type)) {
			deleteMetro(transportationId, authUser);
		} else {
			throw new TransportationException(ErrorCode.INVALID_TRANSPORTATION_TYPE);
		}
	}

	private void deleteBus(Long busId, AuthUser authUser) {
		BusEntity bus = busRepository.findByIdAndDeletedFalse(busId)
			.orElseThrow(() -> new TransportationException(ErrorCode.INVALID_BUS_INFORMATION));

		// 해당 프로필이 현재 사용자의 계정에 속하는지 검증
		if (!bus.getProfile().getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		// 버스 삭제
		bus.delete();
		busRepository.save(bus);

		// 프로필의 대중교통 count 감소
		ProfileEntity profile = bus.getProfile();
		profile.decreaseTransportationCount();
		profileRepository.save(profile);
	}

	private void deleteMetro(Long metroId, AuthUser authUser) {
		MetroEntity metro = metroRepository.findByIdAndDeletedFalse(metroId)
			.orElseThrow(() -> new TransportationException(ErrorCode.METRO_NOT_FOUND));

		// 해당 프로필이 현재 사용자의 계정에 속하는지 검증
		if (!metro.getProfile().getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		// 지하철 삭제
		metro.delete();
		metroRepository.save(metro);

		// 프로필의 대중교통 count 감소
		ProfileEntity profile = metro.getProfile();
		profile.decreaseTransportationCount();
		profileRepository.save(profile);
	}
}
