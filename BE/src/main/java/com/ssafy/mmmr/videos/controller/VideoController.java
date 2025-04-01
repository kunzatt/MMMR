package com.ssafy.mmmr.videos.controller;

import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.videos.dto.VideoRequestDto;
import com.ssafy.mmmr.videos.dto.VideoResponseDto;
import com.ssafy.mmmr.videos.service.VideoService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@RequestMapping("/api/video")
@RequiredArgsConstructor
@Tag(name = "동영상 API", description = "Youtube API를 사용하여 키워드 기반 동영상 id 검색 결과를 반환하는 API")
public class VideoController {

    private final VideoService videoService;

    @Operation(
            summary = "동영상 id 조회",
            description = "키워드를 기반으로 검색된 Youtube 동영상 5개의 id를 반환합니다."
    )
    @ApiResponses({
            @io.swagger.v3.oas.annotations.responses.ApiResponse(
                    responseCode = "200",
                    description = "동영상 id 정보 조회 성공",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ApiResponse.class),
                            examples = @ExampleObject(value = """
                                    {
                                        "message": "동영상 id 정보를 성공적으로 조회했습니다.",
                                        "data": [
                                            { "videoId": "uOMyMt1cz80" },
                                            { "videoId": "52iBKzh3oWU" },
                                            { "videoId": "vp_dzqZXgMg" },
                                            { "videoId": "Ij1-7dMoC18" },
                                            { "videoId": "MuS_YNeYoQ0" }
                                        ]
                                    }
                                    """)
                    )
            ),
            @io.swagger.v3.oas.annotations.responses.ApiResponse(
                    responseCode = "204",
                    description = "해당 키워드로 조회되는 동영상이 존재하지 않습니다.",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ErrorResponse.class),
                            examples = @ExampleObject(value = """
                                    {
                                        "timestamp": "2024-03-20T10:00:00",
                                        "status": 204,
                                        "message": "해당 키워드로 조회되는 동영상이 존재하지 않습니다.",
                                        "errors": []
                                    }
                                    """)
                    )
            ),

            @io.swagger.v3.oas.annotations.responses.ApiResponse(
                    responseCode = "500",
                    description = "서버 오류",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ErrorResponse.class),
                            examples = @ExampleObject(value = """
                                    {
                                        "timestamp": "2024-03-20T10:00:00",
                                        "status": 500,
                                        "message": "동영상 정보를 가져오는 중 오류가 발생했습니다.",
                                        "errors": []
                                    }
                                    """)
                    )
            )
    })
    @PostMapping("/search")
    public ResponseEntity<ApiResponse> searchVideoIdByKeyword(
            @Parameter(description = "동영상 검색 키워드", required = true)
            @io.swagger.v3.oas.annotations.parameters.RequestBody(
                    description = "검색할 키워드 입력",
                    required = true,
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = VideoRequestDto.class),
                            examples = @ExampleObject(
                                    value = """
                                            {
                                                "keyword": "운동 루틴"
                                            }
                                            """
                            )
                    )
            )
            @RequestBody VideoRequestDto request) {
        List<VideoResponseDto> response = videoService.searchVideoIdByKeyword(request.getKeyword());
        return ResponseEntity.ok(new ApiResponse("동영상 api 정보를 성공적으로 조회했습니다.", response));
    }


}
