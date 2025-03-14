package com.ssafy.mmmr.news.controller;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.BusinessException;
import com.ssafy.mmmr.news.dto.NewsContentDto;
import com.ssafy.mmmr.news.dto.NewsTitleDto;
import com.ssafy.mmmr.news.service.NewsService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/api/news")
public class NewsController {

    private final NewsService newsService;

    @Autowired
    public NewsController(NewsService newsService) {
        this.newsService = newsService;
    }

    @Operation(
            summary = "뉴스 제목 목록 반환",
            description = "### 스크래핑된 뉴스 제목 목록을 5개씩 반환합니다. "
    )
    @GetMapping("/title/list")
    ResponseEntity<List<NewsTitleDto>> getNewsTitleList(){
            List<NewsTitleDto> newsTitleDtos = newsService.getNewsTitleList();
            return new ResponseEntity<>(newsTitleDtos, HttpStatus.OK);
    }

    @Operation(
            summary = "특정 뉴스 내용 반환",
            description = "### 특정 뉴스 ID의 스크래핑된 내용을 반환합니다. "
    )
    @GetMapping("/title/content/{id}")
    ResponseEntity<NewsContentDto> getNewsContentById(
            @Parameter(description = "사용자가 선택한 뉴스 제목이 있는 레코드의 ID", example = "1")
            @PathVariable("id") Long id
    ){
        NewsContentDto newsContentDto = newsService.getNewsContentById(id);
        return new ResponseEntity<>(newsContentDto, HttpStatus.OK);
    }



}
