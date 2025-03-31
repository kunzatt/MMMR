package com.ssafy.mmmr.todos.controller;

import java.util.List;
import java.util.Map;

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
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.global.annotation.CurrentUser;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.todos.dto.TodoCreateRequestDto;
import com.ssafy.mmmr.todos.dto.TodoResponseDto;
import com.ssafy.mmmr.todos.dto.TodoUpdateRequestDto;
import com.ssafy.mmmr.todos.service.TodoService;

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
@RequestMapping("/api/todos")
@RequiredArgsConstructor
@Tag(name = "Todo 리스트", description = "투두 생성, 조회, 수정, 삭제 관련 API")
public class TodoController {

	private final TodoService todoService;

	@GetMapping("/profile/{profileId}")
	@Operation(summary = "프로필별 투두 목록 조회", description = "특정 프로필에 속한 모든 투두 목록을 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두 목록을 성공적으로 조회했습니다.",
                        "data": [
                            {
                                "id": 1,
                                "profileId": 1,
                                "content": "스프링 공부하기",
                                "isDone": false,
                                "createdAt": "2024-03-20T10:00:00",
                                "updatedAt": "2024-03-20T10:00:00"
                            },
                            {
                                "id": 2,
                                "profileId": 1,
                                "content": "알고리즘 문제 풀기",
                                "isDone": true,
                                "createdAt": "2024-03-20T11:00:00",
                                "updatedAt": "2024-03-20T12:00:00"
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
		)
	})
	public ResponseEntity<ApiResponse> getTodosByProfile(
		@Parameter(description = "조회할 프로필 ID", required = true) @PathVariable Long profileId,
		@CurrentUser AuthUser authUser) {
		List<TodoResponseDto> todos = todoService.getAllTodosByProfileId(profileId);
		return ResponseEntity.ok(new ApiResponse("투두 목록을 성공적으로 조회했습니다.", todos));
	}

	@GetMapping("/{todoId}")
	@Operation(summary = "투두 상세 조회", description = "특정 투두의 상세 정보를 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두를 성공적으로 조회했습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "content": "스프링 공부하기",
                            "isDone": false,
                            "createdAt": "2024-03-20T10:00:00",
                            "updatedAt": "2024-03-20T10:00:00"
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
                        "message": "해당 투두에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "투두를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "투두를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> getTodo(
		@Parameter(description = "조회할 투두 ID", required = true) @PathVariable Long todoId,
		@CurrentUser AuthUser authUser) {
		TodoResponseDto todo = todoService.getTodo(todoId);
		return ResponseEntity.ok(new ApiResponse("투두를 성공적으로 조회했습니다.", todo));
	}

	@PostMapping
	@Operation(summary = "투두 생성", description = "새로운 투두를 생성합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "201",
			description = "투두 생성 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두가 성공적으로 생성되었습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "content": "스프링 공부하기",
                            "isDone": false,
                            "createdAt": "2024-03-20T10:00:00",
                            "updatedAt": "2024-03-20T10:00:00"
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
                                "field": "content",
                                "message": "내용은 필수 입력 항목입니다"
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
		)
	})
	public ResponseEntity<ApiResponse> addTodo(
		@Parameter(description = "생성할 투두 정보", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "생성할 투두 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = TodoCreateRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "profileId": 1,
                            "content": "스프링 공부하기"
                        }
                        """
				)
			)
		)
		@Valid @RequestBody TodoCreateRequestDto requestDto,
		@CurrentUser AuthUser authUser) {
		TodoResponseDto createdTodo = todoService.addTodo(requestDto);
		return ResponseEntity
			.status(HttpStatus.CREATED)
			.body(new ApiResponse("투두가 성공적으로 생성되었습니다.", createdTodo));
	}

	@PutMapping("/{todoId}")
	@Operation(summary = "투두 내용 수정", description = "특정 투두의 내용을 수정합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 수정 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두가 성공적으로 수정되었습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "content": "스프링 심화 과정 공부하기",
                            "isDone": false,
                            "createdAt": "2024-03-20T10:00:00",
                            "updatedAt": "2024-03-20T11:00:00"
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
                                "field": "content",
                                "message": "내용은 필수 입력 항목입니다"
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
                        "message": "해당 투두에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "투두를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "투두를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> updateTodo(
		@Parameter(description = "수정할 투두 ID", required = true) @PathVariable Long todoId,
		@Parameter(description = "수정할 투두 내용", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "수정할 투두 내용",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = TodoUpdateRequestDto.class),
				examples = @ExampleObject(
					value = """
                        {
                            "content": "스프링 심화 과정 공부하기"
                        }
                        """
				)
			)
		)
		@Valid @RequestBody TodoUpdateRequestDto requestDto,
		@CurrentUser AuthUser authUser) {
		TodoResponseDto updatedTodo = todoService.updateTodo(todoId, requestDto);
		return ResponseEntity.ok(new ApiResponse("투두가 성공적으로 수정되었습니다.", updatedTodo));
	}

	@PutMapping("/{todoId}/toggle")
	@Operation(summary = "투두 완료 상태 토글", description = "특정 투두의 완료 상태를 변경합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 상태 변경 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두 상태가 성공적으로 변경되었습니다.",
                        "data": {
                            "id": 1,
                            "profileId": 1,
                            "content": "스프링 공부하기",
                            "isDone": true,
                            "createdAt": "2024-03-20T10:00:00",
                            "updatedAt": "2024-03-20T11:00:00"
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
                        "message": "해당 투두에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "투두를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "투두를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> toggleTodoStatus(
		@Parameter(description = "상태를 변경할 투두 ID", required = true) @PathVariable Long todoId,
		@CurrentUser AuthUser authUser) {
		TodoResponseDto toggledTodo = todoService.toggleTodoStatus(todoId);
		return ResponseEntity.ok(new ApiResponse("투두 상태가 성공적으로 변경되었습니다.", toggledTodo));
	}

	@DeleteMapping("/{todoId}")
	@Operation(summary = "투두 삭제", description = "특정 투두를 삭제합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 삭제 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "투두가 성공적으로 삭제되었습니다."
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
                        "message": "해당 투두에 대한 권한이 없습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "투두를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 404,
                        "message": "투두를 찾을 수 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> deleteTodo(
		@Parameter(description = "삭제할 투두 ID", required = true) @PathVariable Long todoId,
		@CurrentUser AuthUser authUser) {
		todoService.deleteTodo(todoId);
		return ResponseEntity.ok(new ApiResponse("투두가 성공적으로 삭제되었습니다.", null));
	}

	// TodoController.java에 추가
	@GetMapping("/profile/{profileId}/status")
	@Operation(summary = "프로필별 완료/미완료 투두 목록 조회", description = "특정 프로필에 속한 투두 목록을 완료 상태별로 분리하여 조회합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "투두 목록 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                {
                    "message": "투두 목록을 성공적으로 조회했습니다.",
                    "data": {
                        "completedTodos": [
                            {
                                "id": 2,
                                "profileId": 1,
                                "content": "알고리즘 문제 풀기",
                                "isDone": true,
                                "createdAt": "2024-03-20T11:00:00",
                                "updatedAt": "2024-03-20T12:00:00"
                            }
                        ],
                        "incompleteTodos": [
                            {
                                "id": 1,
                                "profileId": 1,
                                "content": "스프링 공부하기",
                                "isDone": false,
                                "createdAt": "2024-03-20T10:00:00",
                                "updatedAt": "2024-03-20T10:00:00"
                            }
                        ]
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
		)
	})
	public ResponseEntity<ApiResponse> getTodosByProfileAndStatus(
		@Parameter(description = "조회할 프로필 ID", required = true) @PathVariable Long profileId,
		@CurrentUser AuthUser authUser) {
		Map<String, List<TodoResponseDto>> todos = todoService.getTodosByProfileIdAndStatus(profileId);
		return ResponseEntity.ok(new ApiResponse("투두 목록을 성공적으로 조회했습니다.", todos));
	}
}