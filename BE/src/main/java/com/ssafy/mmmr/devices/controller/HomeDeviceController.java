package com.ssafy.mmmr.devices.controller;

import java.util.List;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.devices.dto.HomeDeviceRequestDto;
import com.ssafy.mmmr.devices.dto.HomeDeviceResponseDto;
import com.ssafy.mmmr.devices.service.HomeDeviceService;
import com.ssafy.mmmr.global.annotation.CurrentUser;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/devices")
@RequiredArgsConstructor
@Tag(name = "홈 기기 제어", description = "홈 제어 가능 기기 조회, 추가, 제어, 삭제 관련 API")
public class HomeDeviceController {

	private final HomeDeviceService homeDeviceService;

	@GetMapping
	@Operation(summary = "내 기기 조회", description = "내 계정에 등록된 모든 기기를 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "기기 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "제어 가능한 기기들을 성공적으로 조회했습니다.",
                        "data": [
                            {
                                "id": 1,
                                "accountId": 1,
                                "device": "에어컨",
                                "turned": "OFF"
                            },
                            {
                                "id": 2,
                                "accountId": 1,
                                "device": "TV",
                                "turned": "ON"
                            }
                        ]
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
		)
	})
	public ResponseEntity<ApiResponse> getMyDevices(@CurrentUser AuthUser authUser) {
		List<HomeDeviceResponseDto> devices = homeDeviceService.findByAccountId(authUser.getId());
		return ResponseEntity.ok(new ApiResponse("제어 가능한 기기들을 성공적으로 조회했습니다.", devices));
	}

	@PostMapping
	@Operation(summary = "기기 추가", description = "새로운 제어 가능한 기기를 내 계정에 추가합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "201",
			description = "기기 추가 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "기기를 성공적으로 추가했습니다.",
                        "data": {
                            "id": 3,
                            "accountId": 1,
                            "device": "전등",
                            "turned": "OFF"
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
                        "message": "요청이 올바르지 않습니다",
                        "errors": [
                            {
                                "field": "device",
                                "message": "기기 이름은 필수 입력 항목입니다"
                            }
                        ]
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
		)
	})
	public ResponseEntity<ApiResponse> addDevice(
		@Parameter(description = "추가할 기기 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "추가할 기기 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = HomeDeviceRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "device": "전등",
                            "turned": "OFF"
                        }
                        """
				)
			)
		)
		@Valid @RequestBody HomeDeviceRequestDto requestDto,
		@CurrentUser AuthUser authUser) {
		requestDto.setAccountId(authUser.getId());
		HomeDeviceResponseDto device = homeDeviceService.addDevice(requestDto);
		return new ResponseEntity<>(new ApiResponse("기기를 성공적으로 추가했습니다.", device), HttpStatus.CREATED);
	}

	@PutMapping("/{deviceId}/update")
	@Operation(summary = "기기 상태 변경", description = "기기의 ON/OFF 상태를 변경합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "기기 상태 변경 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "기기를 성공적으로 제어했습니다.",
                        "data": {
                            "id": 1,
                            "accountId": 1,
                            "device": "에어컨",
                            "turned": "ON"
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
                        "message": "해당 기기에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "기기를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "기기를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> toggleDevice(
		@Parameter(description = "상태를 변경할 기기 ID", required = true) @PathVariable Long deviceId, String turned,
		@CurrentUser AuthUser authUser) {
		HomeDeviceResponseDto device = homeDeviceService.updateDevice(deviceId, authUser.getId(), turned);
		return ResponseEntity.ok(new ApiResponse("기기를 성공적으로 제어했습니다.", device));
	}

	@DeleteMapping("/{deviceId}")
	@Operation(summary = "기기 삭제", description = "등록된 기기를 삭제합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "기기 삭제 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "기기를 성공적으로 삭제했습니다."
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
                        "message": "해당 기기에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "기기를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "기기를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> deleteDevice(
		@Parameter(description = "삭제할 기기 ID", required = true) @PathVariable Long deviceId,
		@CurrentUser AuthUser authUser) {
		homeDeviceService.deleteDevice(deviceId, authUser.getId());
		return ResponseEntity.ok(new ApiResponse("기기를 성공적으로 삭제했습니다.", null));
	}
}
