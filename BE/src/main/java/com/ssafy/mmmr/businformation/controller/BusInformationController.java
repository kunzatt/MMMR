package com.ssafy.mmmr.businformation.controller;

import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import com.ssafy.mmmr.businformation.service.BusInformationService;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.error.exception.BusinessException;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@RestController
@RequestMapping("/api/bus-information")
@RequiredArgsConstructor
@Tag(name = "버스 정보 DB 저장 API", description = "엑셀 파일 관련 API")
@Slf4j
public class BusInformationController {

	private final BusInformationService busInformationService;

	@Operation(summary = "버스 정보 Excel 업로드", description = "버스 정보가 담긴 Excel 파일을 업로드하고 DB에 저장합니다.")
	@ApiResponses({
		@ApiResponse(
			responseCode = "200",
			description = "파일 업로드 및 처리 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(type = "object"),
				examples = @ExampleObject(value = """
                    {
                        "success": true,
                        "message": "파일 업로드 및 처리가 완료되었습니다",
                        "importedCount": 10000
                    }
                    """)
			)
		),
		@ApiResponse(
			responseCode = "400",
			description = "잘못된 파일 형식 또는 빈 파일",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = {
					@ExampleObject(
						name = "빈 파일",
						value = """
                            {
                                "timestamp": "2024-03-14T10:00:00",
                                "status": 400,
                                "message": "파일이 비어있습니다",
                                "errors": []
                            }
                            """
					),
					@ExampleObject(
						name = "잘못된 확장자",
						value = """
                            {
                                "timestamp": "2024-03-14T10:00:00",
                                "status": 400,
                                "message": "올바른 확장자 명이 아닙니다",
                                "errors": []
                            }
                            """
					)
				}
			)
		),
		@ApiResponse(
			responseCode = "500",
			description = "서버 오류",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = {
					@ExampleObject(
						name = "파일 처리 오류",
						value = """
                            {
                                "timestamp": "2024-03-14T10:00:00",
                                "status": 500,
                                "message": "파일 업로드 중 오류가 발생했습니다",
                                "errors": []
                            }
                            """
					),
					@ExampleObject(
						name = "엑셀 처리 오류",
						value = """
                            {
                                "timestamp": "2024-03-14T10:00:00",
                                "status": 500,
                                "message": "엑셀 파일 처리 중 오류가 발생했습니다",
                                "errors": []
                            }
                            """
					)
				}
			)
		)
	})
	@PostMapping(value = "", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
	public ResponseEntity<Map<String, Object>> uploadBusInformationFile(
		@Parameter(
			description = "업로드할 버스 정보 Excel 파일",
			required = true,
			content = @Content(mediaType = MediaType.MULTIPART_FORM_DATA_VALUE),
			schema = @Schema(type = "string", format = "binary")
		)
		@RequestParam("file") MultipartFile file
	) throws IOException {
		log.info("버스 정보 Excel 파일 업로드: {}", file.getOriginalFilename());

		// 파일 확장자 검증
		String originalFilename = file.getOriginalFilename();
		if (originalFilename != null && !originalFilename.toLowerCase().endsWith(".xlsx") && !originalFilename.toLowerCase().endsWith(".xls")) {
			throw new BusinessException(ErrorCode.INVALID_EXTENSION_VALUE);
		}

		// 빈 파일 검증
		if (file.isEmpty()) {
			throw new BusinessException(ErrorCode.INVALID_INPUT_VALUE, "파일이 비어있습니다");
		}

		try {
			// 서비스 메서드 호출
			int importedCount = busInformationService.importExcelFile(file);

			// 응답 생성
			Map<String, Object> response = new HashMap<>();
			response.put("success", true);
			response.put("message", "파일 업로드 및 처리가 완료되었습니다");
			response.put("importedCount", importedCount);

			return ResponseEntity.ok(response);
		} catch (Exception e) {
			throw e;
		}
	}
}