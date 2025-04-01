// package com.ssafy.mmmr.transportation.controller;
//
// import java.util.List;
//
// import org.springframework.http.ResponseEntity;
// import org.springframework.web.bind.annotation.DeleteMapping;
// import org.springframework.web.bind.annotation.GetMapping;
// import org.springframework.web.bind.annotation.PathVariable;
// import org.springframework.web.bind.annotation.PostMapping;
// import org.springframework.web.bind.annotation.RequestBody;
// import org.springframework.web.bind.annotation.RequestMapping;
// import org.springframework.web.bind.annotation.RestController;
//
// import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
// import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
// import com.ssafy.mmmr.transportation.service.TransportationService;
//
// import io.swagger.v3.oas.annotations.Operation;
// import io.swagger.v3.oas.annotations.tags.Tag;
// import lombok.RequiredArgsConstructor;
//
// @RestController
// @RequestMapping("/api/transportation")
// @RequiredArgsConstructor
// @Tag(name = "Transportation", description = "교통 정보 API")
// public class TransportationController {
//
// 	private final TransportationService transportationService;
//
// 	@Operation(summary = "지하철 정보 추가", description = "프로필에 지하철 정보를 추가합니다. 프로필당 최대 3개까지 등록 가능합니다.")
// 	@PostMapping("/metro")
// 	public ResponseEntity<Long> addMetro(@RequestBody MetroRequestDto metroRequestDto) {
// 		Long metroId = transportationService.addMetro(metroRequestDto);
// 		return ResponseEntity.ok(metroId);
// 	}
//
// 	@Operation(summary = "지하철 정보 삭제", description = "등록된 지하철 정보를 삭제합니다.")
// 	@DeleteMapping("/metro/{metroId}")
// 	public ResponseEntity<Void> deleteMetro(@PathVariable Long metroId) {
// 		transportationService.deleteMetro(metroId);
// 		return ResponseEntity.noContent().build();
// 	}
//
// 	@Operation(summary = "지하철 도착 정보 조회", description = "프로필에 등록된 지하철 도착 정보를 조회합니다.")
// 	@GetMapping("/metro/arrival/{profileId}")
// 	public ResponseEntity<List<TransportationResponseDto>> getMetroArrivalInfo(@PathVariable Long profileId) {
// 		List<TransportationResponseDto> arrivalInfo = transportationService.getMetroArrivalInfo(profileId);
// 		return ResponseEntity.ok(arrivalInfo);
// 	}
//
// 	@Operation(summary = "모든 교통수단 도착 정보 조회", description = "프로필에 등록된 모든 교통수단(지하철, 버스)의 도착 정보를 조회합니다.")
// 	@GetMapping("/arrival/{profileId}")
// 	public ResponseEntity<List<TransportationResponseDto>> getAllTransportationInfo(@PathVariable Long profileId) {
// 		List<TransportationResponseDto> allInfo = transportationService.getAllTransportationInfo(profileId);
// 		return ResponseEntity.ok(allInfo);
// 	}
// }