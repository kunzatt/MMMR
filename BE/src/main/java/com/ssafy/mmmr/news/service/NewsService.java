package com.ssafy.mmmr.news.service;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.NewsException;
import com.ssafy.mmmr.news.dto.NewsResponseDto;
import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;

@Slf4j
@Service
@RequiredArgsConstructor
public class NewsService {

    private final NewsRepository newsRepository;

    public List<NewsResponseDto> getNewsList() {
            List<NewsEntity> newsEntities = newsRepository.findAll();
            if(newsEntities.isEmpty()) {
                throw new NewsException(ErrorCode.NEWS_FETCH_FAILED);
            }
            List<NewsResponseDto> list = newsEntities.stream()
                    .map(newsEntity -> new NewsResponseDto(
                            newsEntity.getId(),
                            newsEntity.getTitle(),
                            newsEntity.getContent()
                    ))
                    .collect(Collectors.toList());
            return list;
    }

    public void newsCrawler() {
        try{
            String pageUrl = "https://www.yna.co.kr/";
            log.info(pageUrl);
            Document document = Jsoup.connect(pageUrl).userAgent("Mozilla/5.0").timeout(20000).get();
            log.info(document.toString());
            // 메인 기사 : 총 5개
            Elements elements = document.select(".top-main-news001");
            Elements mainArticles = elements.select(".news-con");

            for (Element article : mainArticles) {
                //개별 기사 제목
                String title = article.select(".title01").text().trim();

                // 기사 url
                String articleUrl = article.selectFirst("a.tit-news").attr("href");

                // 기사 내용
                Document contentDocument = Jsoup.connect(articleUrl).timeout(20000).get();
                String content = contentDocument.select(".story-news.article").text();
                //DB에 저장하는 로직
                log.info(title);
                NewsEntity newsEntity = NewsEntity.builder()
                        .id((long)mainArticles.indexOf(article)+1)
                        .title(title)
                        .content(content)
                        .build();
                newsRepository.save(newsEntity);
                log.info("뉴스 크롤링 완료");
            }
        }catch (Exception e){
            log.error(e.getMessage());
            throw new NewsException(ErrorCode.NEWS_CRAWLING_FAILED);
        }
    }

}
