package com.ssafy.mmmr.profiles.controller;

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
import com.ssafy.mmmr.global.annotation.CurrentUser;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.profiles.dto.CallSignResponseDto;
import com.ssafy.mmmr.profiles.dto.ProfileRequestDto;
import com.ssafy.mmmr.profiles.dto.ProfileResponseDto;
import com.ssafy.mmmr.profiles.dto.ProfileUpdateRequestDto;
import com.ssafy.mmmr.profiles.entity.CallSign;
import com.ssafy.mmmr.profiles.service.ProfileService;

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
@RequestMapping("/api/profiles")
@RequiredArgsConstructor
@Tag(name = "프로필 관리", description = "프로필 생성, 조회, 수정, 삭제 관련 API")
public class ProfileController {

	private final ProfileService profileService;

	@PostMapping
	@Operation(summary = "프로필 추가", description = "새로운 프로필을 추가합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "201",
			description = "프로필 추가 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "프로필이 성공적으로 추가되었습니다."
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
	public ResponseEntity<ApiResponse> addProfile(
		@CurrentUser AuthUser authUser,
		@Parameter(description = "추가할 프로필 정보", required = true)
		@RequestBody ProfileRequestDto profileRequestDto) {
		Long accountId = profileService.addProfile(authUser.getId(), profileRequestDto);
		return ResponseEntity
			.status(HttpStatus.CREATED)
			.body(new ApiResponse("프로필이 성공적으로 추가되었습니다.", null));
	}

	@GetMapping
	@Operation(summary = "프로필 목록 조회", description = "사용자의 모든 프로필 목록을 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "프로필 목록을 성공적으로 조회했습니다.",
						"data": [
							{
								"id": 1,
								"nickname": "김용명"
							},
							{
								"id": 2,
								"nickname": "권희주"
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
	public ResponseEntity<ApiResponse> getProfileList(
		@CurrentUser AuthUser authUser) {
		List<ProfileResponseDto> profiles = profileService.getProfileList(authUser.getId());
		return ResponseEntity.ok(new ApiResponse("프로필 목록을 성공적으로 조회했습니다.", profiles));
	}

	@PutMapping("/{profileId}")
	@Operation(summary = "프로필 수정", description = "기존 프로필 정보를 수정합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필 수정 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "프로필이 성공적으로 수정되었습니다."
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "프로필 수정 실패",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "프로필 정보가 올바르지 않습니다",
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
	public ResponseEntity<ApiResponse> updateProfile(
		@Parameter(description = "수정할 프로필 ID", required = true) @PathVariable Long profileId,
		@CurrentUser AuthUser authUser,
		@Parameter(description = "프로필 수정 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "프로필 수정 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ProfileUpdateRequestDto.class),
				examples = @ExampleObject(
					value = """
					{
						"nickname": "string",
						"callSign": "미미"
					}
					"""
				)
			)
		)
		@Valid @RequestBody ProfileUpdateRequestDto updatDto) {
		profileService.updateProfile(profileId, authUser.getId(), updatDto);
		return ResponseEntity.ok(new ApiResponse(("프로필이 성공적으로 수정되었습니다."), null));
	}

	@DeleteMapping("/{profileId}")
	@Operation(summary = "프로필 삭제", description = "지정된 프로필을 삭제합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필 삭제 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "프로필이 성공적으로 삭제되었습니다."
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
	public ResponseEntity<ApiResponse> deleteProfile(
		@Parameter(description = "삭제할 프로필 ID", required = true) @PathVariable Long profileId,
		@CurrentUser AuthUser authUser) {
		profileService.deleteProfile(profileId, authUser.getId());
		return ResponseEntity.ok(new ApiResponse(("프로필이 성공적으로 삭제되었습니다."), null));
	}

	@GetMapping("/callsigns")
	@Operation(summary = "사용 가능한 콜사인 목록 조회", description = "사용자가 사용할 수 있는 콜사인 목록을 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "콜사인 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "사용 가능한 콜사인 목록을 성공적으로 조회했습니다.",
						"data": [
							{
								"id": 1,
								"callSign": "미미"
							},
							{
								"id": 2,
								"callSign": "해태"
							},
							{
								"id": 3,
								"callSign": "루나"
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
	public ResponseEntity<ApiResponse> getAvailableCallSigns(
		@CurrentUser AuthUser authUser) {
		List<CallSignResponseDto> availableCallSigns = profileService.getAvailableCallSigns(authUser.getId());
		return ResponseEntity.ok(new ApiResponse("사용 가능한 콜사인 목록을 성공적으로 조회했습니다.", availableCallSigns));
	}

	@GetMapping("/callsigns/{callSign}")
	@Operation(summary = "콜사인으로 프로필 ID 조회", description = "특정 콜사인에 해당하는 프로필 ID를 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필 ID 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                {
                    "message": "프로필 ID를 성공적으로 조회했습니다.",
                    "data": 1
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
	public ResponseEntity<ApiResponse> getProfileIdByCallSign(
		@Parameter(description = "조회할 콜사인", required = true) @PathVariable CallSign callSign,
		@CurrentUser AuthUser authUser) {
		Long profileId = profileService.getProfileIdByCallSign(callSign, authUser.getId());
		return ResponseEntity.ok(new ApiResponse("콜사인으로 프로필 ID를 성공적으로 조회했습니다.", profileId));
	}
}