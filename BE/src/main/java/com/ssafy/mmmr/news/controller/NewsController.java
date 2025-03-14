package com.ssafy.mmmr.news.controller;

import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.news.dto.NewsContentDto;
import com.ssafy.mmmr.news.dto.NewsTitleDto;
import com.ssafy.mmmr.news.service.NewsService;
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
                            schema = @Schema(implementation = NewsTitleDto.class),
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
    @GetMapping("/title/list")
    ResponseEntity<List<NewsTitleDto>> getNewsTitleList() {
        List<NewsTitleDto> newsTitleDtos = newsService.getNewsTitleList();
        return new ResponseEntity<>(newsTitleDtos, HttpStatus.OK);
    }

    @Operation(
            summary = "특정 뉴스 내용 반환",
            description = "사용자가 선택한 뉴스 제목의 ID를 통해 해당 뉴스의 상세 내용을 반환합니다."
    )
    @ApiResponses({
            @ApiResponse(
                    responseCode = "200",
                    description = "뉴스 내용 조회 성공",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = NewsContentDto.class),
                            examples = @ExampleObject(value = """
                    {
                        "title": "정부의 신재생에너지 확대 발표",
                        "content": "정부는 2024년까지 신재생에너지 비율을 30%로 확대한다고 발표했다..."
                    }
                    """
                            )
                    )
            ),
            @ApiResponse(
                    responseCode = "400",
                    description = "잘못된 요청: ID가 유효하지 않음",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ErrorResponse.class),
                            examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-14T10:00:00",
                        "status": 400,
                        "message": "유효하지 않은 ID입니다",
                        "errors": []
                    }
                    """
                            )
                    )
            ),
            @ApiResponse(
                    responseCode = "404",
                    description = "해당 ID의 뉴스가 존재하지 않음",
                    content = @Content(
                            mediaType = "application/json",
                            schema = @Schema(implementation = ErrorResponse.class),
                            examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-14T10:00:00",
                        "status": 404,
                        "message": "해당 ID의 뉴스를 찾을 수 없습니다",
                        "errors": []
                    }
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
    @GetMapping("/title/content/{id}")
    ResponseEntity<NewsContentDto> getNewsContentById(
            @Parameter(
                    description = "사용자가 선택한 뉴스 제목이 있는 레코드의 ID",
                    example = "1"
            )
            @PathVariable("id") Long id
    ) {
        NewsContentDto newsContentDto = newsService.getNewsContentById(id);
        return new ResponseEntity<>(newsContentDto, HttpStatus.OK);
    }
}


















//package com.ssafy.mmmr.news.controller;
//
//import com.ssafy.mmmr.news.dto.NewsContentDto;
//import com.ssafy.mmmr.news.dto.NewsTitleDto;
//import com.ssafy.mmmr.news.service.NewsService;
//import io.swagger.v3.oas.annotations.Operation;
//import io.swagger.v3.oas.annotations.Parameter;
//import io.swagger.v3.oas.annotations.tags.Tag;
//import lombok.RequiredArgsConstructor;
//import lombok.extern.slf4j.Slf4j;
//import org.springframework.http.HttpStatus;
//import org.springframework.http.ResponseEntity;
//import org.springframework.web.bind.annotation.*;
//
//import java.util.List;
//
//@RestController
//@RequestMapping("/api/news")
//@RequiredArgsConstructor
//@Tag(name = "버스 정보 DB 저장 API", description = "엑셀 파일 관련 API")
//@Slf4j
//public class NewsController {
//
//    private final NewsService newsService;
//
//    @Operation(
//            summary = "뉴스 제목 목록 반환",
//            description = "### 스크래핑된 뉴스 제목 목록을 5개씩 반환합니다. "
//    )
//    @GetMapping("/title/list")
//    ResponseEntity<List<NewsTitleDto>> getNewsTitleList(){
//            List<NewsTitleDto> newsTitleDtos = newsService.getNewsTitleList();
//            return new ResponseEntity<>(newsTitleDtos, HttpStatus.OK);
//    }
//
//    @Operation(
//            summary = "특정 뉴스 내용 반환",
//            description = "### 특정 뉴스 ID의 스크래핑된 내용을 반환합니다. "
//    )
//    @GetMapping("/title/content/{id}")
//    ResponseEntity<NewsContentDto> getNewsContentById(
//            @Parameter(description = "사용자가 선택한 뉴스 제목이 있는 레코드의 ID", example = "1")
//            @PathVariable("id") Long id
//    ){
//        NewsContentDto newsContentDto = newsService.getNewsContentById(id);
//        return new ResponseEntity<>(newsContentDto, HttpStatus.OK);
//    }
//
//
//
//}
