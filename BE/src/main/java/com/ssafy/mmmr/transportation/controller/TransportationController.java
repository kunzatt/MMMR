package com.ssafy.mmmr.transportation.controller;

import java.util.List;

import org.springframework.http.HttpStatus;
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
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.transportation.dto.TransportationProfileResponseDto;
import com.ssafy.mmmr.transportation.dto.TransportationRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
import com.ssafy.mmmr.transportation.dto.TransportationSearchResponseDto;
import com.ssafy.mmmr.transportation.service.TransportationService;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/transportations")
@RequiredArgsConstructor
@Tag(name = "교통 API", description = "교통 정보 API")
public class TransportationController {

	private final TransportationService transportationService;

	@GetMapping("/search")
	@Operation(summary = "대중교통 검색", description = "버스와 지하철 정보를 검색합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "대중교통 검색 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "대중교통 검색 성공",
                        "data": [
                            {
                                "type": "BUS",
                                "station": "강남역",
                                "sequence": "10",
                                "number": "5002",
                                "information": "stationId: 13241, routeId: 32145"
                            },
                            {
                                "type": "METRO",
                                "station": "강남역",
                                "sequence": "",
                                "number": "2호선",
                                "information": "지하철 2호선"
                            }
                        ]
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> searchTransportation(
		@Parameter(description = "검색 유형 (BUS, METRO, ALL)", required = false)
		@RequestParam(required = false, defaultValue = "ALL") String type,
		@Parameter(description = "검색 키워드", required = false)
		@RequestParam(required = false) String keyword) {

		List<TransportationSearchResponseDto> results = transportationService.searchTransportation(type, keyword);

		return ResponseEntity.ok(new ApiResponse("대중교통 검색 성공", results));
	}

	@PostMapping
	@Operation(summary = "대중교통 정보 추가", description = "프로필에 버스 또는 지하철 정보를 추가합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "대중교통 정보 추가 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "대중교통 정보 추가 성공",
                        "data": {
                            "id": 1,
                            "profile": {
                                "id": 1
                            },
                            "routeId": "32145",
                            "route": "5002",
                            "stationId": "13241",
                            "station": "강남역",
                            "direction": ""
                        }
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "잘못된 요청",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 400,
                        "message": "유효하지 않은 교통 정보입니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "인증 정보 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 401,
                        "message": "인증 정보가 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "403",
			description = "권한 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 403,
                        "message": "해당 프로필에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> addTransportation(
		@Parameter(description = "추가할 대중교통 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "추가할 대중교통 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = TransportationRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "profileId": 1,
                            "type": "BUS",
                            "number": "040",
                            "station": "강남역12번출구",
                            "routeId": "104000014",
                            "stationId": "122000181"
                        }
                        """
				)
			)
		)
		@RequestBody TransportationRequestDto requestDto,
		@CurrentUser AuthUser authUser) {

		Object result = transportationService.addTransportation(requestDto, authUser);
		return ResponseEntity.status(HttpStatus.CREATED)
			.body(new ApiResponse("대중교통 정보 추가 성공", result));
	}

	@DeleteMapping("/{transportationId}")
	@Operation(summary = "대중교통 정보 삭제", description = "특정 대중교통 정보를 삭제합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "대중교통 정보 삭제 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "대중교통 정보가 성공적으로 삭제되었습니다.",
                        "data": null
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "인증 정보 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 401,
                        "message": "인증 정보가 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "403",
			description = "권한 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 403,
                        "message": "해당 교통정보에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "교통정보를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "교통정보를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> deleteTransportation(
		@Parameter(description = "삭제할 교통정보 ID", required = true) @PathVariable Long transportationId,
		@Parameter(description = "교통 유형 (BUS, METRO)", required = true) @RequestParam String type,
		@CurrentUser AuthUser authUser) {

		transportationService.deleteTransportation(transportationId, type, authUser);
		return ResponseEntity.ok(new ApiResponse("대중교통 정보가 성공적으로 삭제되었습니다.", null));
	}

	@GetMapping("/profile/{profileId}")
	@Operation(summary = "프로필 교통 정보 조회", description = "프로필에 저장된 모든 버스 및 지하철 정보를 조회합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필 교통 정보 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "프로필 교통 정보 조회 성공",
                        "data": {
                            "buses": [
                                {
                                    "id": 1,
                                    "type": "BUS",
                                    "route": "5002",
                                    "station": "강남역",
                                    "routeId": "32145",
                                    "stationId": "13241",
                                    "direction": "양재역"
                                }
                            ],
                            "metros": [
                                {
                                    "id": 2,
                                    "type": "METRO",
                                    "line": 2,
                                    "station": "강남역"
                                }
                            ],
                            "totalCount": 2
                        }
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "인증 정보 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 401,
                        "message": "인증 정보가 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "403",
			description = "권한 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 403,
                        "message": "해당 프로필에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "프로필을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "프로필을 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> getProfileTransportations(
		@Parameter(description = "조회할 프로필 ID", required = true) @PathVariable Long profileId,
		@CurrentUser AuthUser authUser) {

		TransportationProfileResponseDto result = transportationService.getTransportationsByProfile(profileId, authUser);
		return ResponseEntity.ok(new ApiResponse("프로필 교통 정보 조회 성공", result));
	}

	@GetMapping("/profile/{profileId}/arrivals")
	@Operation(summary = "프로필에 저장된 모든 교통수단 도착 정보 조회",
		description = "프로필에 저장된 모든 버스와 지하철의 실시간 도착 정보를 조회합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "도착 정보 조회 성공",
			content = @Content(
				mediaType = "application/json",
				examples = @ExampleObject(value = """
                    [
                        {
                            "id": 1,
                            "type": "BUS",
                            "number": "5002",
                            "station": "강남역",
                            "direction": "양재역 방향",
                            "information": "2분 30초 후, 5정거장 전"
                        },
                        {
                            "id": 2,
                            "type": "METRO",
                            "number": "2호선",
                            "station": "강남역",
                            "direction": "신도림방면",
                            "information": "곧 도착, 4분 후"
                        }
                    ]
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "프로필을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "프로필을 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<List<TransportationResponseDto>> getAllProfileTransportationArrivals(
		@Parameter(description = "조회할 프로필 ID", required = true) @PathVariable Long profileId
	) {
		List<TransportationResponseDto> arrivals = transportationService.getTransportationArrivalsByProfile(profileId);
		return ResponseEntity.ok(arrivals);
	}
}