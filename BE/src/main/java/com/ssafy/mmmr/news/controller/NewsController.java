package com.ssafy.mmmr.news.controller;

import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.news.dto.NewsResponseDto;
import com.ssafy.mmmr.news.service.NewsService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/api/news")
@RequiredArgsConstructor
@Tag(name = "뉴스 API", description = "뉴스 크롤링 데이터 제공 API")
@Slf4j
public class NewsController {

    private final NewsService newsService;

    @Operation(
            summary = "뉴스 제목 목록 반환",
            description = "스크래핑된 뉴스 제목 목록을 5개씩 반환합니다."
    )
    @ApiResponses({
            @ApiResponse(
                    responseCode = "200",
                    description = "성공적으로 뉴스 제목 목록을 반환",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = NewsResponseDto.class),
                            examples = @ExampleObject(value = """
                    [
                        {
                            "id": 1,
                            "title": "정부, 신재생에너지 확대 발표"
                        },
                        {
                            "id": 2,
                            "title": "AI 기술 발전, 산업 구조 변화 예고"
                        },
                        {
                            "id": 3,
                            "title": "제목3"
                        },
                        {
                            "id": 4,
                            "title": "제목 4"
                        },
                        {
                            "id": 5,
                            "title": "제목 5"
                        }
                    ]
                    """
                            )
                    )
            ),
            @ApiResponse(
                    responseCode = "500",
                    description = "서버 오류 발생",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ErrorResponse.class),
                            examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-14T10:00:00",
                        "status": 500,
                        "message": "서버 오류로 인해 데이터를 불러올 수 없습니다",
                        "errors": []
                    }
                    """
                            )
                    )
            )
    })
    @GetMapping
    ResponseEntity<List<NewsResponseDto>> getNewsTitleList() {
        List<NewsResponseDto> newsResponseDtos = newsService.getNewsList();
        return new ResponseEntity<>(newsResponseDtos, HttpStatus.OK);
    }
}
