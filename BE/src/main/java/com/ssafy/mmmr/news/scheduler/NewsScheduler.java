package com.ssafy.mmmr.news.scheduler;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.BusinessException;
import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import lombok.extern.slf4j.Slf4j;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.time.LocalDateTime;

@Slf4j
@Component
public class NewsScheduler {

    private final NewsRepository newsRepository;

    public NewsScheduler(NewsRepository newsRepository) {

        this.newsRepository = newsRepository;
    }

    @Scheduled(cron = "0 0 * * * *")
    public void newsScheduler() throws IOException {

        try{
            String pageUrl = "https://www.yna.co.kr/";

            Document document = Jsoup.connect(pageUrl).timeout(5000).get();
            // 메인 기사 : 총 5개
            Elements elements = document.select(".top-main-news001");
            Elements mainArticles = elements.select(".news-con");

            for (Element article : mainArticles) {
                //개별 기사 제목
                String title = article.select(".title01").text().trim();

                // 기사 url
                String articleUrl = article.selectFirst("a.tit-news").attr("href");

                // 기사 내용
                Document contentDocument = Jsoup.connect(articleUrl).timeout(5000).get();
                String content = contentDocument.select(".story-news.article").text();
                //DB에 저장하는 로직
                NewsEntity newsEntity = NewsEntity.builder()
                        .id((long)mainArticles.indexOf(article)+1)
                        .title(title)
                        .content(content)
                        .created_at(LocalDateTime.now())
                        .build();
                newsRepository.save(newsEntity);
                log.info("뉴스 크롤링 완료");

            }

        }catch (Exception e){
            log.error(e.getMessage());
            throw new BusinessException(ErrorCode.INTERNAL_SERVER_ERROR, "뉴스 크롤링 실패");
        }


    }
}
