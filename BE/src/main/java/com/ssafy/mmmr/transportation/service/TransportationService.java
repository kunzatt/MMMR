package com.ssafy.mmmr.transportation.service;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.repository.BusInformationRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.ProfileException;
import com.ssafy.mmmr.global.error.exception.TransportationException;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.transportation.client.BusClient;
import com.ssafy.mmmr.transportation.client.MetroClient;
import com.ssafy.mmmr.transportation.dto.TransportationProfileResponseDto;
import com.ssafy.mmmr.transportation.dto.TransportationRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
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
	private final MetroClient metroClient;
	private final BusClient busClient;

	public List<TransportationSearchResponseDto> searchTransportation(String type, String keyword) {
		List<TransportationSearchResponseDto> results = new ArrayList<>();

		if ("ALL".equalsIgnoreCase(type) || "BUS".equalsIgnoreCase(type)) {
			List<BusInformationEntity> busResults = busInformationRepository.searchByKeyword(keyword);

			results.addAll(busResults.stream()
				.map(bus -> TransportationSearchResponseDto.builder()
					.type("BUS")
					.station(bus.getStation())
					.sequence(String.valueOf(bus.getSequence()))
					.number(bus.getRoute())
					.information("stationId: " + bus.getStationId() + ", routeId: " + bus.getRouteId())
					.build())
				.collect(Collectors.toList()));
		}

		if ("ALL".equalsIgnoreCase(type) || "METRO".equalsIgnoreCase(type)) {
			List<MetroInformationEntity> metroResults = metroInformationRepository.searchByKeyword(keyword);

			results.addAll(metroResults.stream()
				.map(metro -> TransportationSearchResponseDto.builder()
					.type("METRO")
					.station(metro.getStationName() + "역")
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
		ProfileEntity profile = profileRepository.findById(requestDto.getProfileId())
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (!profile.getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		if (profile.getCount() >= 3) {
			throw new TransportationException(ErrorCode.MORE_THAN_THREE_TRANSPORTATION);
		}

		if ("BUS".equalsIgnoreCase(requestDto.getType())) {
			return addBus(profile, requestDto);
		} else if ("METRO".equalsIgnoreCase(requestDto.getType())) {
			return addMetro(profile, requestDto);
		} else {
			throw new TransportationException(ErrorCode.INVALID_TRANSPORTATION_TYPE);
		}
	}

	private BusEntity addBus(ProfileEntity profile, TransportationRequestDto requestDto) {
		if (requestDto.getNumber() == null || requestDto.getStation() == null ||
			requestDto.getRouteId() == null || requestDto.getStationId() == null) {
			throw new TransportationException(ErrorCode.INVALID_BUS_INFORMATION);
		}

		BusInformationEntity currentBusInfo = busInformationRepository
			.findByRouteIdAndStationId(
				requestDto.getRouteId(),
				requestDto.getStationId())
			.orElseThrow(() -> new TransportationException(ErrorCode.INVALID_BUS_INFORMATION));

		BusInformationEntity nextBusInfo = busInformationRepository
			.findByRouteIdAndSequence(
				requestDto.getRouteId(),
				currentBusInfo.getSequence() + 1)
			.orElse(null);

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

		String stationName = requestDto.getStation() + "역";

		MetroInformationEntity metroInfo = metroInformationRepository
			.findByFlexibleLineNumberAndStationName(requestDto.getNumber(), requestDto.getStation())
			.orElseThrow(() -> new TransportationException(ErrorCode.INVALID_STATION_NAME));

		Integer lineNumber = Integer.parseInt(
			metroInfo.getLineNumber().replaceAll("[^0-9]", "")
		);

		MetroEntity metroEntity = MetroEntity.builder()
			.profile(profile)
			.line(lineNumber)
			.station(stationName)
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

		if (!bus.getProfile().getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		bus.delete();
		busRepository.save(bus);

		ProfileEntity profile = bus.getProfile();
		profile.decreaseTransportationCount();
		profileRepository.save(profile);
	}

	private void deleteMetro(Long metroId, AuthUser authUser) {
		MetroEntity metro = metroRepository.findByIdAndDeletedFalse(metroId)
			.orElseThrow(() -> new TransportationException(ErrorCode.METRO_NOT_FOUND));

		if (!metro.getProfile().getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		metro.delete();
		metroRepository.save(metro);

		ProfileEntity profile = metro.getProfile();
		profile.decreaseTransportationCount();
		profileRepository.save(profile);
	}

	@Transactional(readOnly = true)
	public TransportationProfileResponseDto getTransportationsByProfile(Long profileId, AuthUser authUser) {
		ProfileEntity profile = profileRepository.findById(profileId)
			.orElseThrow(() -> new ProfileException(ErrorCode.PROFILE_NOT_FOUND));

		if (!profile.getAccount().getEmail().equals(authUser.getEmail())) {
			throw new ProfileException(ErrorCode.UNAUTHORIZED);
		}

		List<BusEntity> buses = busRepository.findByProfileIdAndDeletedFalse(profileId);

		List<MetroEntity> metros = metroRepository.findByProfileIdAndDeletedFalse(profileId);

		List<TransportationProfileResponseDto.BusInfo> busInfoList = buses.stream()
			.map(bus -> TransportationProfileResponseDto.BusInfo.builder()
				.id(bus.getId())
				.type("BUS")
				.route(bus.getRoute())
				.station(bus.getStation())
				.routeId(String.valueOf(bus.getRouteId()))
				.stationId(String.valueOf(bus.getStationId()))
				.direction(bus.getDirection())
				.build())
			.collect(Collectors.toList());

		List<TransportationProfileResponseDto.MetroInfo> metroInfoList = metros.stream()
			.map(metro -> TransportationProfileResponseDto.MetroInfo.builder()
				.id(metro.getId())
				.type("METRO")
				.line(String.valueOf(metro.getLine()))
				.station(metro.getStation())
				.build())
			.collect(Collectors.toList());

		return TransportationProfileResponseDto.builder()
			.buses(busInfoList)
			.metros(metroInfoList)
			.totalCount(buses.size() + metros.size())
			.build();
	}

	@Transactional(readOnly = true)
	public List<TransportationResponseDto> getMetroArrivalsByProfile(Long profileId) {
		List<MetroEntity> savedMetros = metroRepository.findByProfileIdAndDeletedFalse(profileId);

		List<TransportationResponseDto> allArrivals = new ArrayList<>();

		for (MetroEntity metro : savedMetros) {
			String stationName = metro.getStation();
			if (stationName.endsWith("역")) {
				stationName = stationName.substring(0, stationName.length() - 1);
			}

			List<TransportationResponseDto> stationArrivals = metroClient.getMetroArrivals(stationName, metro.getId());

			String targetLineNumber = metro.getLine() + "호선";

			String mappedLine = mapLineNumber(metro.getLine());

			List<TransportationResponseDto> filteredArrivals = stationArrivals.stream()
				.filter(arrival -> {
					return arrival.getNumber().equals(targetLineNumber) ||
						arrival.getNumber().contains(mappedLine);
				})
				.collect(Collectors.toList());

			if (filteredArrivals.isEmpty() && !stationArrivals.isEmpty()) {
				TransportationResponseDto defaultArrival = TransportationResponseDto.builder()
					.id(metro.getId())
					.type("METRO")
					.number(targetLineNumber)
					.station(metro.getStation())
					.direction("정보 없음")
					.information("현재 도착 정보가 없습니다.")
					.build();

				allArrivals.add(defaultArrival);
			} else {
				allArrivals.addAll(filteredArrivals);
			}
		}

		return allArrivals;
	}

	@Transactional(readOnly = true)
	public List<TransportationResponseDto> getBusArrivalsByProfile(Long profileId) {
		List<BusEntity> savedBuses = busRepository.findByProfileIdAndDeletedFalse(profileId);

		List<TransportationResponseDto> allArrivals = new ArrayList<>();

		for (BusEntity bus : savedBuses) {
			String routeId = String.valueOf(bus.getRouteId());
			String stationId = String.valueOf(bus.getStationId());
			String busNumber = bus.getRoute();
			String stationName = bus.getStation();
			String direction = bus.getDirection();

			List<TransportationResponseDto> busArrivals = busClient.getBusArrivals(
				routeId, stationId, busNumber, stationName, direction, bus.getId());

			allArrivals.addAll(busArrivals);
		}

		return allArrivals;
	}

	@Transactional(readOnly = true)
	public List<TransportationResponseDto> getTransportationArrivalsByProfile(Long profileId) {
		List<TransportationResponseDto> allArrivals = new ArrayList<>();

		List<TransportationResponseDto> metroArrivals = getMetroArrivalsByProfile(profileId);
		allArrivals.addAll(metroArrivals);

		List<TransportationResponseDto> busArrivals = getBusArrivalsByProfile(profileId);
		allArrivals.addAll(busArrivals);

		return allArrivals;
	}

	private String mapLineNumber(Integer lineNumber) {
		switch (lineNumber) {
			case 1: return "1호선";
			case 2: return "2호선";
			case 3: return "3호선";
			case 4: return "4호선";
			case 5: return "5호선";
			case 6: return "6호선";
			case 7: return "7호선";
			case 8: return "8호선";
			case 9: return "9호선";
			case 10: return "경의중앙선";
			case 11: return "공항철도";
			case 12: return "경춘선";
			case 13: return "수인분당선";
			case 14: return "신분당선";
			case 15: return "우이신설선";
			default: return lineNumber + "호선";
		}
	}
}
