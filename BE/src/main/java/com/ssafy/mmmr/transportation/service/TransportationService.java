package com.ssafy.mmmr.transportation.service;

import java.util.ArrayList;
import java.util.List;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;

import com.fasterxml.jackson.dataformat.xml.XmlMapper;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.TransportationException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.transportation.dto.MetroApiResponseDto;
import com.ssafy.mmmr.transportation.dto.MetroApiResponseDto.MetroArrivalInfo;
import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.repository.BusRepository;
import com.ssafy.mmmr.transportation.repository.MetroRepository;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Slf4j
@Service
@RequiredArgsConstructor
public class TransportationService {

	private final ProfileRepository profileRepository;
	private final MetroRepository metroRepository;
	private final BusRepository busRepository;
	private final RestTemplate restTemplate;
	private final XmlMapper xmlMapper;

	@Value("${seoul.metro.api.url}")
	private String metroApiUrl;

	@Value("${seoul.metro.api.key}")
	private String metroApiKey;

	@Transactional
	public Long addMetro(MetroRequestDto metroRequestDto) {
		ProfileEntity profile = profileRepository.findById(metroRequestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		int activeMetroCount = metroRepository.countActiveMetrosByProfile(profile);
		int activeBusCount = busRepository.countActiveBusesByProfile(profile);

		if (activeMetroCount + activeBusCount >= 3) {
			throw new TransportationException(ErrorCode.MORE_THAN_THREE_TRANSPORTATION);
		}

		MetroEntity metro = MetroEntity.builder()
			.profile(profile)
			.line(metroRequestDto.getLine())
			.station(metroRequestDto.getStation())
			.direction(metroRequestDto.getDirection())
			.build();

		return metroRepository.save(metro).getId();
	}

	@Transactional
	public void deleteMetro(Long metroId) {
		MetroEntity metro = metroRepository.findById(metroId)
			.orElseThrow(() -> new TransportationException(ErrorCode.METRO_NOT_FOUND));
		metro.delete();
	}

	/**
	 * 프로필에 등록된 지하철 도착 정보를 조회합니다.
	 *
	 * @param profileId 프로필 ID
	 * @return 지하철 도착 정보 목록
	 */
	@Transactional(readOnly = true)
	public List<TransportationResponseDto> getMetroArrivalInfo(Long profileId) {
		ProfileEntity profile = profileRepository.findById(profileId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		List<MetroEntity> metros = metroRepository.findActiveMetrosByProfile(profile);
		List<TransportationResponseDto> results = new ArrayList<>();

		for (MetroEntity metro : metros) {
			try {
				// 서울 지하철 실시간 도착정보 API 호출
				String apiUrl = UriComponentsBuilder.fromHttpUrl(metroApiUrl)
					.path("/{apiKey}/xml/realtimeStationArrival/0/20/{stationName}")
					.buildAndExpand(metroApiKey, metro.getStation())
					.toUriString();

				log.debug("지하철 API 호출: {}", apiUrl);

				// API 호출 및 응답 파싱
				String response = restTemplate.getForObject(apiUrl, String.class);
				MetroApiResponseDto metroResponse = xmlMapper.readValue(response, MetroApiResponseDto.class);

				if (metroResponse != null && metroResponse.getRows() != null) {
					log.debug("지하철 도착정보 응답: {} 개의 결과", metroResponse.getRows().size());

					// 호선, 방향에 맞는 도착 정보 필터링
					List<MetroArrivalInfo> matchingArrivals = new ArrayList<>();
					for (MetroArrivalInfo arrival : metroResponse.getRows()) {
						if (isMatchingMetroInfo(metro, arrival)) {
							matchingArrivals.add(arrival);
						}
					}

					// 최대 2개까지만 정보 추가
					int count = 0;
					for (MetroArrivalInfo arrival : matchingArrivals) {
						if (count >= 2) break;  // 최대 2개까지만 표시

						TransportationResponseDto dto = TransportationResponseDto.builder()
							.type("METRO")
							.station(metro.getStation())
							.number(String.valueOf(metro.getLine()))
							.sequence("")
							.information(formatMetroArrivalInfo(arrival))
							.build();
						results.add(dto);
						count++;
					}

					// 조회된 결과가 없을 경우 정보 없음 메시지 추가
					if (count == 0) {
						TransportationResponseDto dto = TransportationResponseDto.builder()
							.type("METRO")
							.station(metro.getStation())
							.number(String.valueOf(metro.getLine()))
							.sequence("")
							.information("도착 정보 없음")
							.build();
						results.add(dto);
					}
				}
			} catch (Exception e) {
				log.error("지하철 도착정보 조회 오류 (역: {}): {}", metro.getStation(), e.getMessage(), e);

				// 에러 발생 시에도 응답에 정보 추가
				TransportationResponseDto dto = TransportationResponseDto.builder()
					.type("METRO")
					.station(metro.getStation())
					.number(String.valueOf(metro.getLine()))
					.sequence("")
					.information("도착 정보를 조회할 수 없습니다")
					.build();
				results.add(dto);
			}
		}

		return results;
	}

	@Transactional(readOnly = true)
	public List<TransportationResponseDto> getAllTransportationInfo(Long profileId) {
		List<TransportationResponseDto> results = new ArrayList<>();
		results.addAll(getMetroArrivalInfo(profileId));
		// 버스 기능이 구현되면 아래 주석을 해제
		// results.addAll(getBusArrivalInfo(profileId));
		return results;
	}

	private boolean isMatchingMetroInfo(MetroEntity metro, MetroArrivalInfo arrival) {
		// 호선 확인 (subwayId와 line 비교)
		boolean lineMatches = String.valueOf(metro.getLine()).equals(arrival.getSubwayId());

		// 방향 확인 (지정된 방향이 없으면 모든 방향 허용)
		boolean directionMatches = true;

		if (metro.getDirection() != null && !metro.getDirection().isEmpty()) {
			// 상행/하행 또는 외선/내선 체크
			if (metro.getDirection().equals("상행") || metro.getDirection().equals("하행") ||
				metro.getDirection().equals("외선") || metro.getDirection().equals("내선")) {
				directionMatches = metro.getDirection().equals(arrival.getUpdnLine());
			}
			// 특정 방면 체크
			else {
				directionMatches = arrival.getTrainLineNm().contains(metro.getDirection());
			}
		}

		return lineMatches && directionMatches;
	}

	private String formatMetroArrivalInfo(MetroArrivalInfo arrival) {
		String trainType = "일반".equals(arrival.getBtrainSttus()) ? "" : "[" + arrival.getBtrainSttus() + "] ";
		return String.format("%s%s (%s)",
			trainType,
			arrival.getTrainLineNm(),
			arrival.getArvlMsg2());
	}
}