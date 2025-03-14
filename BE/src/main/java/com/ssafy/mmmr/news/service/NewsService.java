package com.ssafy.mmmr.news.service;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.BusinessException;
import com.ssafy.mmmr.news.dto.NewsContentDto;
import com.ssafy.mmmr.news.dto.NewsTitleDto;
import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

@Slf4j
@Service
public class NewsService {

    private final NewsRepository newsRepository;

    @Autowired
    public NewsService(
            NewsRepository newsRepository
    ) {
        this.newsRepository = newsRepository;
    }

    public List<NewsTitleDto> getNewsTitleList() {
        try {
            List<NewsEntity> newsEntities = newsRepository.findAll();

            assert !newsEntities.isEmpty() : "newsEntities is null";
            return newsEntities.stream()
                    .map(newsEntity -> new NewsTitleDto(
                            newsEntity.getId(),
                            newsEntity.getTitle()
                    ))
                    .collect(Collectors.toList());

        }catch (Exception e){
            log.error("getNewsTitleList : {}", e.getMessage());
            throw new BusinessException(ErrorCode.INTERNAL_SERVER_ERROR, "뉴스 제목 조회 실패");
        }

    }

    public NewsContentDto getNewsContentById(Long id) {
        try {
            Optional<NewsEntity> tmp = newsRepository.findById(id);
            NewsEntity newsEntity = tmp.orElse(null);

            assert newsEntity != null : "newsEntity is null";
            return NewsContentDto.builder()
                    .title(newsEntity.getTitle())
                    .content(newsEntity.getContent())
                    .build();

        }catch (Exception e){
            log.error("getNewsContentById : {}", e.getMessage());
            throw new BusinessException(ErrorCode.INTERNAL_SERVER_ERROR, "뉴스 내용 조회 실패");
        }
    }

}
