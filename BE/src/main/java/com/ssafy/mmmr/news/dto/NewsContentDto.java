package com.ssafy.mmmr.news.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class NewsContentDto {

    @Schema(description = "뉴스 ID", example = "1")
    private long id;

    @Schema(description = "뉴스 제목", example = "'주주에 대한 이사 충실의무' 상법 개정안, 野주도 국회 통과")
    private String title;

    @Schema(description = "뉴스 내용", example = "(요약되지 않는 내용 전부)")
    private String content;
}
