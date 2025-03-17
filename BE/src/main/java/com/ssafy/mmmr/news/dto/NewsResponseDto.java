package com.ssafy.mmmr.news.dto;

import lombok.*;

@Getter
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class NewsResponseDto {

    private Long id;

    private String title;

    private String content;
}
