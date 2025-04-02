package com.ssafy.mmmr.transportation.controller;

import java.util.List;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.global.annotation.CurrentUser;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationSearchResponseDto;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.service.TransportationService;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/transportation")
@RequiredArgsConstructor
@Tag(name = "Transportation", description = "교통 정보 API")
public class TransportationController {

	private final TransportationService transportationService;

	@GetMapping("/search")
	@Operation(summary = "대중교통 검색", description = "버스와 지하철 정보를 검색합니다")
	public ResponseEntity<ApiResponse> searchTransportation(
		@RequestParam(required = false, defaultValue = "ALL") String type,
		@RequestParam(required = false) String keyword) {

		List<TransportationSearchResponseDto> results = transportationService.searchTransportation(type, keyword);

		return ResponseEntity.ok(new ApiResponse("대중교통 검색 성공", results));
	}

	@PostMapping("/add")
	@Operation(summary = "대중교통 정보 추가", description = "프로필에 버스 또는 지하철 정보를 추가합니다")
	public ResponseEntity<ApiResponse> addTransportation(
		@RequestBody TransportationRequestDto requestDto,
		@CurrentUser AuthUser authUser) {

		Object result = transportationService.addTransportation(requestDto, authUser);
		ApiResponse response = new ApiResponse("대중교통 정보 추가 성공", result);
		return ResponseEntity.ok(response);
	}

	@DeleteMapping("/{transportationId}")
	@Operation(summary = "대중교통 정보 삭제", description = "특정 대중교통 정보를 삭제합니다")
	public ResponseEntity<ApiResponse> deleteTransportation(
		@PathVariable Long transportationId,
		@RequestParam String type,
		@CurrentUser AuthUser authUser) {

		transportationService.deleteTransportation(transportationId, type, authUser);
		ApiResponse response = new ApiResponse("대중교통 정보가 성공적으로 삭제되었습니다.", null);
		return ResponseEntity.ok(response);
	}
}