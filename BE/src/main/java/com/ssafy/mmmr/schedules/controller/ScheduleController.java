package com.ssafy.mmmr.schedules.controller;

import java.util.List;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.global.annotation.CurrentUser;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.schedules.dto.ScheduleRequestDto;
import com.ssafy.mmmr.schedules.dto.ScheduleResponseDto;
import com.ssafy.mmmr.schedules.service.ScheduleService;

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
@RequestMapping("/api/schedules")
@RequiredArgsConstructor
@Tag(name = "Schedule 관리", description = "일정 생성, 조회, 수정, 삭제 관련 API")
public class ScheduleController {

	private final ScheduleService scheduleService;

	@GetMapping("/{id}")
	@Operation(summary = "특정 일정 조회", description = "ID로 특정 일정을 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "일정 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "일정을 성공적으로 조회했습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "title": "스프링 스터디",
                            "startDate": "2024-03-20 14:00:00",
                            "endDate": "2024-03-20 16:00:00",
                            "createdAt": "2024-03-20 10:00:00",
                            "updatedAt": "2024-03-20 10:00:00"
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
                        "message": "해당 일정에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "일정을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "일정을 찾을 수 없습니다.",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> getScheduleById(
		@Parameter(description = "조회할 일정 ID", required = true) @PathVariable Long id,
		@CurrentUser AuthUser authUser) {
		ScheduleResponseDto responseDto = scheduleService.getScheduleById(id, authUser);
		return ResponseEntity.ok(new ApiResponse("일정을 성공적으로 조회했습니다.", responseDto));
	}

	@PostMapping
	@Operation(summary = "일정 추가", description = "새로운 일정을 추가합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "201",
			description = "일정 추가 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "일정이 성공적으로 추가되었습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "title": "스프링 스터디",
                            "startDate": "2024-03-20 14:00:00",
                            "endDate": "2024-03-20 16:00:00",
                            "createdAt": "2024-03-20 10:00:00",
                            "updatedAt": "2024-03-20 10:00:00"
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
                        "message": "종료 날짜는 시작 날짜보다 이후여야 합니다.",
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
                        "message": "프로필을 찾을 수 없습니다.",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> createSchedule(
		@Parameter(description = "추가할 일정 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "추가할 일정 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ScheduleRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "profileId": 1,
                            "title": "스프링 스터디",
                            "startDate": "2024-03-20T14:00:00",
                            "endDate": "2024-03-20T16:00:00"
                        }
                        """
				)
			)
		)
		@Valid @RequestBody ScheduleRequestDto requestDto,
		@CurrentUser AuthUser authUser) {
		ScheduleResponseDto responseDto = scheduleService.addSchedule(requestDto, authUser);
		return ResponseEntity
			.status(HttpStatus.CREATED)
			.body(new ApiResponse("일정이 성공적으로 추가되었습니다.", responseDto));
	}

	@GetMapping("/profile")
	@Operation(summary = "프로필별 일정 목록 조회", description = "특정 프로필에 속한 모든 일정 목록을 조회하거나 키워드로 검색합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "프로필별 일정 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "프로필별 일정 목록을 성공적으로 조회했습니다.",
                        "data": [
                            {
                                "id": 1,
                                "profileId": 1,
                                "title": "스프링 스터디",
                                "startDate": "2024-03-20 14:00:00",
                                "endDate": "2024-03-20 16:00:00",
                                "createdAt": "2024-03-20 10:00:00",
                                "updatedAt": "2024-03-20 10:00:00"
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
                        "message": "프로필을 찾을 수 없습니다.",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> getSchedulesByProfile(
		@Parameter(description = "조회할 프로필 ID", required = true) @RequestParam Long profileId,
		@Parameter(description = "검색 키워드 (선택사항)") @RequestParam(required = false) String keyword,
		@CurrentUser AuthUser authUser) {

		List<ScheduleResponseDto> responseDtos;

		if (keyword != null && !keyword.isEmpty()) {
			responseDtos = scheduleService.searchSchedulesByProfileAndKeyword(profileId, keyword, authUser);
			return ResponseEntity.ok(new ApiResponse("키워드에 해당하는 일정을 성공적으로 검색했습니다.", responseDtos));
		} else {
			responseDtos = scheduleService.getSchedulesByProfile(profileId, authUser);
			return ResponseEntity.ok(new ApiResponse("프로필별 일정 목록을 성공적으로 조회했습니다.", responseDtos));
		}
	}

	@PutMapping("/{id}")
	@Operation(summary = "일정 수정", description = "특정 일정의 정보를 부분 수정합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "일정 수정 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "일정이 성공적으로 수정되었습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "title": "스프링 스터디 (변경됨)",
                            "startDate": "2024-03-20 15:00:00",
                            "endDate": "2024-03-20 17:00:00",
                            "createdAt": "2024-03-20 10:00:00",
                            "updatedAt": "2024-03-20 11:00:00"
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
                        "message": "종료 날짜는 시작 날짜보다 이후여야 합니다.",
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
                        "message": "해당 일정에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "일정을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "일정을 찾을 수 없습니다.",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> updateSchedule(
		@Parameter(description = "수정할 일정 ID", required = true) @PathVariable Long id,
		@Parameter(description = "수정할 일정 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "수정할 일정 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ScheduleRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "profileId": 1,
                            "title": "스프링 스터디 (변경됨)",
                            "startDate": "2024-03-20T15:00:00",
                            "endDate": "2024-03-20T17:00:00"
                        }
                        """
				)
			)
		)
		@Valid @RequestBody ScheduleRequestDto requestDto,
		@CurrentUser AuthUser authUser) {
		ScheduleResponseDto responseDto = scheduleService.updateSchedule(id, requestDto, authUser);
		return ResponseEntity.ok(new ApiResponse("일정이 성공적으로 수정되었습니다.", responseDto));
	}

	@DeleteMapping("/{id}")
	@Operation(summary = "일정 삭제", description = "특정 일정을 삭제합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "204",
			description = "일정 삭제 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "일정이 성공적으로 삭제되었습니다."
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
                        "message": "해당 일정에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "일정을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "일정을 찾을 수 없습니다.",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> deleteSchedule(
		@Parameter(description = "삭제할 일정 ID", required = true) @PathVariable Long id,
		@CurrentUser AuthUser authUser) {
		scheduleService.deleteSchedule(id, authUser);
		return ResponseEntity.status(HttpStatus.NO_CONTENT)
			.body(new ApiResponse("일정이 성공적으로 삭제되었습니다.", null));
	}
}